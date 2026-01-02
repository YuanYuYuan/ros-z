use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering::AcqRel;
use std::time::Duration;
use std::{marker::PhantomData, sync::Arc};
use std::collections::VecDeque;

use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait, sample::{Locality, Sample}};
use zenoh_ext::{AdvancedPublisher, CacheConfig, AdvancedPublisherBuilderExt, AdvancedSubscriberBuilderExt, HistoryConfig, RecoveryConfig};

use crate::Builder;
use crate::attachment::{Attachment, GidArray};
use crate::entity::EndpointEntity;
use crate::event::EventsManager;
use crate::impl_with_type_info;
use crate::topic_name;

use crate::msg::{CdrSerdes, ZDeserializer, ZMessage, ZSerializer};
use crate::qos::{QosDurability, QosHistory, QosProfile, QosReliability};
use std::sync::Mutex;

// KeepLastQueue implementation for ROS KEEP_LAST semantics
pub struct KeepLastQueue<T> {
    queue: Mutex<VecDeque<T>>,
    capacity: usize,
}

impl<T> KeepLastQueue<T> {
    pub fn new(capacity: usize) -> Self {
        Self {
            queue: Mutex::new(VecDeque::with_capacity(capacity)),
            capacity,
        }
    }

    /// Push with ROS KEEP_LAST semantics: drop oldest if full
    pub fn push(&self, item: T) {
        let mut q = self.queue.lock().unwrap();
        if q.len() == self.capacity {
            q.pop_front();  // Drop oldest
        }
        q.push_back(item);
    }

    pub fn recv(&self) -> std::result::Result<T, RecvError> {
        let mut q = self.queue.lock().unwrap();
        q.pop_front().ok_or(RecvError::Empty)
    }

    pub fn recv_timeout(&self, timeout: Duration) -> std::result::Result<T, RecvTimeoutError> {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            {
                let mut q = self.queue.lock().unwrap();
                if let Some(item) = q.pop_front() {
                    return Ok(item);
                }
            }
            if std::time::Instant::now() >= deadline {
                return Err(RecvTimeoutError::Timeout);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    pub async fn recv_async(&self) -> std::result::Result<T, RecvError> {
        loop {
            {
                let mut q = self.queue.lock().unwrap();
                if let Some(item) = q.pop_front() {
                    return Ok(item);
                }
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    }

    pub fn try_recv(&self) -> std::result::Result<T, TryRecvError> {
        let mut q = self.queue.lock().unwrap();
        q.pop_front().ok_or(TryRecvError::Empty)
    }

    pub fn len(&self) -> usize {
        self.queue.lock().unwrap().len()
    }

    pub fn is_empty(&self) -> bool {
        self.queue.lock().unwrap().is_empty()
    }
}

#[derive(Debug)]
pub enum RecvError {
    Empty,
}

impl std::fmt::Display for RecvError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RecvError::Empty => write!(f, "Queue is empty"),
        }
    }
}

impl std::error::Error for RecvError {}

#[derive(Debug)]
pub enum RecvTimeoutError {
    Timeout,
}

impl std::fmt::Display for RecvTimeoutError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RecvTimeoutError::Timeout => write!(f, "Receive timeout"),
        }
    }
}

impl std::error::Error for RecvTimeoutError {}

#[derive(Debug)]
pub enum TryRecvError {
    Empty,
}

impl std::fmt::Display for TryRecvError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TryRecvError::Empty => write!(f, "Queue is empty"),
        }
    }
}

impl std::error::Error for TryRecvError {}

pub struct ZPub<T: ZMessage, S: ZSerializer> {
    pub entity: EndpointEntity,
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: AdvancedPublisher<'static>,
    _lv_token: LivelinessToken,
    with_attachment: bool,
    events_mgr: Arc<Mutex<EventsManager>>,
    _phantom_data: PhantomData<(T, S)>,
}

impl<T: ZMessage, S: ZSerializer> Drop for ZPub<T, S> {
    fn drop(&mut self) {
        tracing::debug!(
            "ZPub::drop: Dropping publisher for topic={}, type={:?}, id={}",
            self.entity.topic,
            self.entity.type_info.as_ref().map(|t| &t.name),
            self.entity.id
        );
    }
}

#[derive(Debug)]
pub struct ZPubBuilder<T, S = CdrSerdes<T>> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub with_attachment: bool,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl_with_type_info!(ZPubBuilder<T, S>);
impl_with_type_info!(ZSubBuilder<T, S>);

impl<T, S> ZPubBuilder<T, S> {
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.entity.qos = qos;
        self
    }

    pub fn with_attachment(mut self, with_attachment: bool) -> Self {
        self.with_attachment = with_attachment;
        self
    }

    pub fn with_serdes<S2>(self) -> ZPubBuilder<T, S2> {
        ZPubBuilder {
            entity: self.entity,
            session: self.session,
            with_attachment: self.with_attachment,
            _phantom_data: PhantomData,
        }
    }
}

impl<T, S> Builder for ZPubBuilder<T, S>
where
    T: ZMessage + 'static,
    S: for<'a> ZSerializer<Input<'a> = &'a T> + 'static,
{
    type Output = ZPub<T, S>;

    fn build(mut self) -> Result<Self::Output> {
        // Qualify the topic name according to ROS 2 rules
        let qualified_topic = topic_name::qualify_topic_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

        self.entity.topic = qualified_topic;

        let key_expr = self.entity.topic_key_expr()?;
        tracing::debug!("[PUB] KE: {key_expr}");

        // Map QoS to Zenoh publisher settings
        let mut pub_builder = self.session.declare_publisher(key_expr);

        // Map reliability and congestion control
        match self.entity.qos.reliability {
            QosReliability::Reliable => {
                pub_builder = pub_builder
                    .reliability(zenoh::qos::Reliability::Reliable)
                    .congestion_control(zenoh::qos::CongestionControl::Block);
            }
            QosReliability::BestEffort => {
                pub_builder = pub_builder
                    .reliability(zenoh::qos::Reliability::BestEffort)
                    .congestion_control(zenoh::qos::CongestionControl::Drop);
            }
        }

        // Map durability: TransientLocal uses cache for persistence
        let inner = match self.entity.qos.durability {
            QosDurability::TransientLocal => {
                let depth = match self.entity.qos.history {
                    QosHistory::KeepLast(d) => d.get(),
                    QosHistory::KeepAll => usize::MAX,
                };
                let mut builder = pub_builder
                    .advanced()
                    .cache(CacheConfig::default().max_samples(depth))
                    .publisher_detection();

                // Only enable sample_miss_detection for RELIABLE + TRANSIENT_LOCAL
                // This matches rmw_zenoh_cpp behavior and uses SequenceNumber sequencing
                // to avoid requiring timestamping in Zenoh config
                if self.entity.qos.reliability == QosReliability::Reliable {
                    builder = builder.sample_miss_detection(
                        zenoh_ext::MissDetectionConfig::default()
                            .sporadic_heartbeat(Duration::from_millis(500))
                    );
                }
                builder.wait()?
            }
            QosDurability::Volatile => {
                // For Volatile: use advanced publisher without cache or publisher_detection
                // C++ doesn't set publisher_detection for Volatile
                pub_builder
                    .advanced()
                    .wait()?
            }
        };
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;
        let gid = self.entity.gid();
        Ok(ZPub {
            entity: self.entity,
            sn: AtomicUsize::new(0),
            inner,
            _lv_token: lv_token,
            gid,
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
            with_attachment: self.with_attachment,
            _phantom_data: Default::default(),
        })
    }
}

impl<T, S> ZPub<T, S>
where
    T: ZMessage + 'static,
    S: for<'a> ZSerializer<Input<'a> = &'a T> + 'static,
{
    fn new_attchment(&self) -> Attachment {
        Attachment::new(self.sn.fetch_add(1, AcqRel) as _, self.gid)
    }

    pub fn publish(&self, msg: &T) -> Result<()> {
        eprintln!("[ZPub::publish] Serializing message");
        let serialized = S::serialize(msg);
        eprintln!("[ZPub::publish] Creating put_builder");
        let mut put_builder = self.inner.put(serialized);
        if self.with_attachment {
            eprintln!("[ZPub::publish] Adding attachment");
            put_builder = put_builder.attachment(self.new_attchment());
        }
        eprintln!("[ZPub::publish] Calling wait() - TESTING if it works without tokio");
        let result = put_builder.wait();
        eprintln!("[ZPub::publish] wait() completed: {:?}", result.is_ok());
        result
    }

    pub async fn async_publish(&self, msg: &T) -> Result<()> {
        let mut put_builder = self.inner.put(S::serialize(msg));
        if self.with_attachment {
            put_builder = put_builder.attachment(self.new_attchment());
        }
        put_builder.await
    }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        let mut put_builder = self.inner.put(msg);
        if self.with_attachment {
            put_builder = put_builder.attachment(self.new_attchment());
        }
        put_builder.wait()
    }

    pub fn publish_sample(&self, msg: &Sample) -> Result<()> {
        let mut put_builder = self.inner.put(msg.payload().to_bytes());
        if self.with_attachment {
            put_builder = put_builder.attachment(self.new_attchment());
        }
        put_builder.wait()
    }

    pub fn events_mgr(&self) -> &Arc<Mutex<EventsManager>> {
        &self.events_mgr
    }
}

pub struct ZSubBuilder<T, S = CdrSerdes<T>> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub ignore_local_publications: bool,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl<T, S> ZSubBuilder<T, S>
where
    T: ZMessage,
{
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.entity.qos = qos;
        self
    }

    pub fn ignore_local_publications(mut self, ignore: bool) -> Self {
        self.ignore_local_publications = ignore;
        self
    }

    pub fn with_serdes<S2>(self) -> ZSubBuilder<T, S2> {
        ZSubBuilder {
            entity: self.entity,
            session: self.session,
            ignore_local_publications: self.ignore_local_publications,
            _phantom_data: PhantomData,
        }
    }

    /// Build a subscriber with a callback that processes deserialized messages directly.
    ///
    /// This method creates a subscriber that invokes the provided callback for each
    /// received message, bypassing the internal queue. The callback receives the
    /// deserialized message directly. Liveliness tokens and event management are
    /// preserved.
    ///
    /// # Arguments
    ///
    /// * `callback` - A function that will be called with each deserialized message
    ///
    /// # Returns
    ///
    /// A `ZSub` with no internal queue (callback-only mode)
    pub fn build_with_callback<F>(mut self, callback: F) -> Result<ZSub<T, (), S>>
    where
        F: Fn(S::Output) + Send + Sync + 'static,
        S: for<'a> ZDeserializer<Input<'a> = &'a [u8]> + 'static,
    {
        // Qualify the topic name according to ROS 2 rules
        let qualified_topic = topic_name::qualify_topic_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

        self.entity.topic = qualified_topic;

        // Get queue size for history config
        let queue_size = match self.entity.qos.history {
            QosHistory::KeepLast(depth) => depth.get(),
            QosHistory::KeepAll => 1000,
        };

        // Always use AdvancedSubscriber, configure history for TRANSIENT_LOCAL
        let mut adv_sub_builder = self
            .session
            .declare_subscriber(self.entity.topic_key_expr()?)
            .advanced();

        // For TRANSIENT_LOCAL durability, configure history
        if self.entity.qos.durability == QosDurability::TransientLocal {
            adv_sub_builder = adv_sub_builder
                .subscriber_detection()
                .history(HistoryConfig::default()
                    .detect_late_publishers()
                    .max_samples(queue_size));

            // Enable recovery for RELIABLE + TRANSIENT_LOCAL
            if self.entity.qos.reliability == QosReliability::Reliable {
                adv_sub_builder = adv_sub_builder.recovery(RecoveryConfig::default());
            }
        }

        if self.ignore_local_publications {
            adv_sub_builder = adv_sub_builder.allowed_origin(Locality::Remote);
        }

        let inner = adv_sub_builder
            .callback(move |sample| {
                dbg!();
                let payload = sample.payload().to_bytes();
                dbg!();
                let msg = S::deserialize(&payload);
                dbg!();
                callback(msg);
                dbg!();
            })
            .wait()?;

        // Declare liveliness token to preserve graph presence
        let gid = self.entity.gid();
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;

        Ok(ZSub {
            entity: self.entity,
            queue: None,
            _inner: inner,
            _lv_token: lv_token,
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
            _phantom_data: Default::default(),
        })
    }

    #[cfg(feature = "rmw-z")]
    pub fn build_with_notifier<F>(mut self, notify: F) -> Result<ZSub<T, Sample, S>>
    where
        F: Fn() + Send + Sync + 'static,
        S: ZDeserializer,
    {
        // Qualify the topic name according to ROS 2 rules
        let qualified_topic = topic_name::qualify_topic_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

        self.entity.topic = qualified_topic;

        // Map QoS history to queue size
        let queue_size = match self.entity.qos.history {
            QosHistory::KeepLast(depth) => depth.get(),
            QosHistory::KeepAll => 1000, // Use a reasonable default for KeepAll
        };

        // Create KeepLastQueue instead of flume
        let queue = Arc::new(KeepLastQueue::new(queue_size));
        let queue_clone = queue.clone();

        // Always use AdvancedSubscriber, configure history for TRANSIENT_LOCAL
        let mut adv_sub_builder = self
            .session
            .declare_subscriber(self.entity.topic_key_expr()?)
            .advanced();

        // For TRANSIENT_LOCAL durability, configure history
        if self.entity.qos.durability == QosDurability::TransientLocal {
            adv_sub_builder = adv_sub_builder
                .subscriber_detection()
                .history(HistoryConfig::default()
                    .detect_late_publishers()
                    .max_samples(queue_size));

            // Enable recovery for RELIABLE + TRANSIENT_LOCAL
            if self.entity.qos.reliability == QosReliability::Reliable {
                adv_sub_builder = adv_sub_builder.recovery(RecoveryConfig::default());
            }
        }

        if self.ignore_local_publications {
            adv_sub_builder = adv_sub_builder.allowed_origin(Locality::Remote);
        }

        let inner = adv_sub_builder
            .callback(move |sample| {
                queue_clone.push(sample);
                notify();
            })
            .wait()?;
        let gid = self.entity.gid();
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;
        Ok(ZSub {
            entity: self.entity,
            _inner: inner,
            _lv_token: lv_token,
            queue: Some(queue),
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
            _phantom_data: Default::default(),
        })
    }
}

impl<T, S> Builder for ZSubBuilder<T, S>
where
    T: ZMessage + 'static + Sync + Send,
    S: ZDeserializer,
{
    type Output = ZSub<T, Sample, S>;

    fn build(mut self) -> Result<Self::Output> {
        // Qualify the topic name according to ROS 2 rules
        let qualified_topic = topic_name::qualify_topic_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

        self.entity.topic = qualified_topic;

        // Map QoS history to queue size
        let queue_size = match self.entity.qos.history {
            QosHistory::KeepLast(depth) => depth.get(),
            QosHistory::KeepAll => 1000, // Use a reasonable default for KeepAll
        };

        // Create KeepLastQueue instead of flume
        let queue = Arc::new(KeepLastQueue::new(queue_size));
        let queue_clone = queue.clone();

        // Always use AdvancedSubscriber, configure history for TRANSIENT_LOCAL
        let mut adv_sub_builder = self
            .session
            .declare_subscriber(self.entity.topic_key_expr()?)
            .advanced();

        // For TRANSIENT_LOCAL durability, configure history
        if self.entity.qos.durability == QosDurability::TransientLocal {
            adv_sub_builder = adv_sub_builder
                .subscriber_detection()
                .history(HistoryConfig::default()
                    .detect_late_publishers()
                    .max_samples(queue_size));

            // Enable recovery for RELIABLE + TRANSIENT_LOCAL
            if self.entity.qos.reliability == QosReliability::Reliable {
                adv_sub_builder = adv_sub_builder.recovery(RecoveryConfig::default());
            }
        }

        if self.ignore_local_publications {
            adv_sub_builder = adv_sub_builder.allowed_origin(Locality::Remote);
        }

        let inner = adv_sub_builder
            .callback(move |sample| {
                queue_clone.push(sample);
            })
            .wait()?;
        let gid = self.entity.gid();
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;
        Ok(Self::Output {
            entity: self.entity,
            _inner: inner,
            _lv_token: lv_token,
            queue: Some(queue),
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
            _phantom_data: Default::default(),
        })
    }
}

pub struct ZSub<T: ZMessage, Q, S: ZDeserializer> {
    pub entity: EndpointEntity,
    pub queue: Option<Arc<KeepLastQueue<Q>>>,
    _inner: zenoh_ext::AdvancedSubscriber<()>,
    _lv_token: LivelinessToken,
    events_mgr: Arc<Mutex<EventsManager>>,
    _phantom_data: PhantomData<(T, S)>,
}

impl<T, S> ZSub<T, Sample, S>
where
    T: ZMessage,
    S: ZDeserializer,
{
    /// Receive the next serialized message (raw sample)
    pub fn recv_serialized(&self) -> Result<Sample> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let msg = queue.recv()
            .map_err(|e| zenoh::Error::from(format!("Queue recv error: {}", e)))?;
        Ok(msg)
    }

    /// Async receive the next serialized message (raw sample)
    pub async fn async_recv_serialized(&self) -> Result<Sample> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let msg = queue.recv_async().await
            .map_err(|e| zenoh::Error::from(format!("Queue recv error: {}", e)))?;
        Ok(msg)
    }

    pub fn events_mgr(&self) -> &Arc<Mutex<EventsManager>> {
        &self.events_mgr
    }
}

impl<T, S> ZSub<T, Sample, S>
where
    T: ZMessage,
    S: for<'a> ZDeserializer<Input<'a> = &'a [u8]>,
{
    /// Receive and deserialize the next message (aligned with ROS behavior)
    pub fn recv(&self) -> Result<S::Output> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let sample = queue.recv()
            .map_err(|e| zenoh::Error::from(format!("Queue recv error: {}", e)))?;
        let payload = sample.payload().to_bytes();
        Ok(S::deserialize(&payload))
    }

    pub fn recv_timeout(&self, timeout: Duration) -> Result<S::Output> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let sample = queue.recv_timeout(timeout)
            .map_err(|e| zenoh::Error::from(format!("Queue recv timeout error: {}", e)))?;
        let payload = sample.payload().to_bytes();
        Ok(S::deserialize(&payload))
    }

    /// Async receive and deserialize the next message
    pub async fn async_recv(&self) -> Result<S::Output> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let sample = queue.recv_async().await
            .map_err(|e| zenoh::Error::from(format!("Queue recv error: {}", e)))?;
        let payload = sample.payload().to_bytes();
        Ok(S::deserialize(&payload))
    }
}
