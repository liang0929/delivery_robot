declare module 'roslib' {
  export class Ros {
    constructor(options: { url: string });
    on(event: 'connection' | 'error' | 'close', callback: (error?: Error) => void): void;
    close(): void;
  }

  export class Topic {
    constructor(options: {
      ros: Ros;
      name: string;
      messageType: string;
    });
    publish(message: Message): void;
    subscribe(callback: (message: unknown) => void): void;
    unsubscribe(): void;
  }

  export class Message {
    constructor(values: Record<string, unknown>);
  }
}
