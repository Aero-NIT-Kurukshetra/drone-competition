import os
import json
from typing import Dict, Callable, Awaitable, Any
from redis import asyncio as aioredis
import asyncio
import time
import logging

logging.basicConfig(
    level=logging.DEBUG if __name__ == "__main__" else logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger(__name__)

class RedisClient:
    """Shared Redis client for inter-process communication"""
    def __init__(self, loop = None, worker_id: str = "worker_1"):
        self.__host = os.getenv('REDIS_HOST', 'panel.silentjungle.tech')
        self.__port = int(os.getenv('REDIS_PORT', 50009))
        self.__username = os.getenv('REDIS_USERNAME', None)
        self.__password = os.getenv('REDIS_PASSWORD', 'DAMkMn9g6WMZaHqK29L4VxNQW7gFYpgu')
        self.__loop = loop or asyncio.get_event_loop()
        self.worker_id = worker_id
        
        self.client: aioredis.Redis | None = None
        self.pubsub_client: aioredis.Redis | None = None
        self.listeners: Dict[str, Callable[[Any], Awaitable[None]]] = {}

        self.init = False

    async def connect(self):
        """Connect to Redis server and setup pub/sub listeners"""
        while True:
            try:
                self.client = aioredis.Redis(
                    host=self.__host,
                    port=self.__port,
                    username=self.__username,
                    password=self.__password,
                    db=0,
                    decode_responses=True,
                    socket_connect_timeout=15,
                    socket_keepalive=True,
                    health_check_interval=30
                )
                await self.client.ping()
                logger.info(f"[RedisClient] Connected to Redis at {self.__host}:{self.__port}")

                self.pubsub_client = aioredis.Redis(
                    host=self.__host,
                    port=self.__port,
                    username=self.__username,
                    password=self.__password,
                    db=0,
                    decode_responses=True,
                    socket_connect_timeout=15,
                    socket_keepalive=True,
                    health_check_interval=30
                )
                await self.pubsub_client.ping()
                logger.info(f"[RedisClient] Pub/Sub client connected to Redis at {self.__host}:{self.__port}")

                # send state
                await self.client.set(f"state:{self.worker_id}", json.dumps({
                    "status": "running",
                    "timestamp": time.time()
                }))

                if self.listeners:
                    self.__loop.create_task(self.pubsub_loop())

                return # exit the connect loop on success
            except Exception as e:
                logger.error(f"[RedisClient] Failed to connect to Redis: {e}")
                await asyncio.sleep(2) # wait before retrying

    async def pubsub_loop(self):
        while True:
            try:
                pubsub = self.pubsub_client.pubsub()
                for channel, callback in self.listeners.items():
                    await pubsub.subscribe(channel)
                    logger.info(f"[RedisClient] Subscribed to channel '{channel}'")

                async for message in pubsub.listen():
                    if message['type'] == 'message':
                        channel = message['channel']
                        # data = json.loads(message['data'])
                        data = message['data']

                        try:
                            data = json.loads(data)
                        except json.JSONDecodeError:
                            pass

                        callback = self.listeners.get(channel, None)
                        if callback:
                            logging.debug(f"[RedisClient] Message received on channel '{channel}': {data}")
                            try:
                                # await callback(data)
                                asyncio.create_task(callback(data)) # run in separate task to avoid blocking
                            except Exception as e:
                                logger.error(f"[RedisClient] Error in listener callback for channel '{channel}': {e}")
            except Exception as e:
                logger.error(f"[RedisClient] Error in listener loop: {e}")
                await asyncio.sleep(2)  # wait before retrying

    async def get_startup_mode(self) -> str:
        """Retrieve the startup mode from Redis"""
        if not self.client:
            raise RuntimeError("Redis client is not connected")

        mode = await self.client.get(f"startup_mode:{self.worker_id}")
        if mode is None:
            return "normal"  # default mode
        return mode
    
    async def close(self):
        """Close Redis connection"""
        if self.client:
            await self.client.set(f"state:{self.worker_id}", json.dumps({
                "status": "stopped",
                "timestamp": time.time()
            }))

            await self.client.aclose()
            logger.info("[RedisClient] Connection closed")

    def add_listener(self, channel: str, callback: Callable[[Any], Awaitable[None]]):
        """
        Add a listener for a specific Redis channel
        This only registers the listener if ran before connecting to Redis.

        Args:
            channel: Channel name to subscribe to
            callback: Async function to call when a message is received
        """
        if channel in self.listeners:
            logger.error(f"[RedisClient] Listener for channel '{channel}' already exists")
            return

        self.listeners[channel] = callback

    def remove_listener(self, channel: str):
        """
        Remove a listener for a specific Redis channel

        Args:
            channel: Channel name to unsubscribe from
        """
        if channel in self.listeners:
            del self.listeners[channel]

    def listen(self, channel: str):
        """
        Decorator to register a listener for a specific Redis channel
        
        Args:
            channel: Channel name to subscribe to
        """
        def decorator(func: Callable[[Any], Awaitable[None]]):
            self.add_listener(channel, func)
            return func
        return decorator
    
    async def publish(self, channel: str, message: dict):
        """
        Publish a message to a specific Redis channel

        Args:
            channel: Channel name to publish to
            message: Message dictionary to send
        """
        if not self.client:
            raise RuntimeError("Redis client is not connected")

        await self.pubsub_client.publish(channel, json.dumps(message))
        logger.debug(f"[RedisClient] Published message to channel '{channel}': {message}")

    async def heartbeat(self):
        """
        Send a heartbeat message for a specific worker

        Args:
            worker_id: Unique identifier for the worker
            status: Current status of the worker
        """
        heartbeat_data = {
            'worker_id': self.worker_id,
            'timestamp': time.time()
        }
        
        try:
            await self.client.set(f'heartbeat:{self.worker_id}', json.dumps(heartbeat_data), ex = 3)
        except Exception as e:
            logger.error(f"[RedisClient] Failed to send heartbeat: {e}")

# for testing purposes
if __name__ == "__main__":
    from dotenv import load_dotenv

    load_dotenv()
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    db = RedisClient(loop = loop)

    @db.listen('test_channel')
    async def handle_test_message(message):
        print(f"Received message on test_channel: {message}")
        print(type(message))
    
    async def test_redis_client():
        await db.connect()
        await db.client.set('test_key', 'test_value')
        value = await db.client.get('test_key')
        print(f"Retrieved from Redis: {value}")

        await db.publish('test_channel', {'foo': 'bar', 'number': 42})
        # await db.close()

    async def heartbeat_loop():
        while True:
            await db.heartbeat()
            
            if not db.init:
                db.init = True

            await asyncio.sleep(1)

    loop.create_task(test_redis_client())
    try:
        loop.run_until_complete(heartbeat_loop())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Error in main loop: {e}")
        loop.run_until_complete(db.close())
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()