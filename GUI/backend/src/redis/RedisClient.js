import { createClient } from "redis";
import process from "process";

export default class RedisClient {
    constructor({ workerId = "worker_1" } = {}) {
        this.host = process.env.REDIS_HOST || "panel.silentjungle.tech";
        this.port = process.env.REDIS_PORT || 50009;
        this.username = process.env.REDIS_USERNAME && process.env.REDIS_USERNAME.trim() !== ""
            ? process.env.REDIS_USERNAME
            : undefined;
        this.password = process.env.REDIS_PASSWORD || "DAMkMn9g6WMZaHqK29L4VxNQW7gFYpgu";

        this.workerId = workerId;

        this.client = createClient({
            socket: {
                host: this.host,
                port: this.port
            },
            password: this.password,
            username: this.username
        });
        this.subscriber = null;

        this.listeners = new Map();
        this.init = false;
    }

    async connect() {
        while (true) {
            try {
                this.client = createClient({
                    socket: {
                        host: this.host,
                        port: this.port,
                        reconnectStrategy: (retries) => Math.min(retries * 100, 2000),
                    },
                    username: this.username,
                    password: this.password,
                });

                this.client.on("error", (err) => {
                    console.error("[RedisClient] Redis error:", err);
                });

                await this.client.connect();
                await this.client.ping();

                console.log(
                    `[RedisClient] Connected to Redis at ${this.host}:${this.port}`
                );

                await this.client.set(
                    `state:${this.workerId}`,
                    JSON.stringify({
                        status: "running",
                        timestamp: Date.now() / 1000,
                    })
                );

                if (this.listeners.size > 0) {
                    this.pubsubLoop();
                }

                return;
            } catch (err) {
                console.error("[RedisClient] Failed to connect:", err);
                await new Promise((r) => setTimeout(r, 2000));
            }
        }
    }

    async pubsubLoop() {
        this.subscriber = this.client.duplicate();
        await this.subscriber.connect();

        for (const channel of this.listeners.keys()) {
            await this.subscriber.subscribe(channel, (message) => {
                let data = message;
                try {
                    data = JSON.parse(message);
                } catch (_) { }

                const callback = this.listeners.get(channel);
                if (callback) {
                    Promise.resolve(callback(data)).catch((err) => {
                        console.error(
                            `[RedisClient] Listener error on channel ${channel}:`,
                            err
                        );
                    });
                }
            });

            console.log(`[RedisClient] Subscribed to channel '${channel}'`);
        }
    }

    listen(channel, callback) {
        if (this.listeners.has(channel)) {
            throw new Error(
                `[RedisClient] Listener already exists for ${channel}`
            );
        }
        this.listeners.set(channel, callback);
    }

    removeListener(channel) {
        this.listeners.delete(channel);
    }

    async publish(channel, message) {
        if (!this.client) {
            throw new Error("Redis client not connected");
        }

        await this.client.publish(channel, JSON.stringify(message));
    }

    async getStartupMode() {
        if (!this.client) {
            throw new Error("Redis client not connected");
        }

        const mode = await this.client.get(
            `startup_mode:${this.workerId}`
        );
        return mode ?? "normal";
    }

    async heartbeat() {
        if (!this.client) return;

        const payload = {
            worker_id: this.workerId,
            timestamp: Date.now() / 1000,
        };

        try {
            await this.client.set(
                `heartbeat:${this.workerId}`,
                JSON.stringify(payload),
                { EX: 3 }
            );
        } catch (err) {
            console.error("[RedisClient] Heartbeat failed:", err);
        }
    }

    async close() {
        if (!this.client) return;

        await this.client.set(
            `state:${this.workerId}`,
            JSON.stringify({
                status: "stopped",
                timestamp: Date.now() / 1000,
            })
        );

        await this.client.quit();
        if (this.subscriber) await this.subscriber.quit();

        console.log("[RedisClient] Connection closed");
    }
}
