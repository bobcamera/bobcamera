/**
 * Type declarations for roslib
 * Based on roslib.js API
 */

declare module 'roslib' {
  export interface RosOptions {
    url: string
    transportLibrary?: string
    transportOptions?: any
  }

  export class Ros {
    constructor(options: RosOptions)
    
    isConnected: boolean
    
    on(event: 'connection', callback: () => void): void
    on(event: 'error', callback: (error: any) => void): void
    on(event: 'close', callback: () => void): void
    
    close(): void
  }

  export interface TopicOptions {
    ros: Ros
    name: string
    messageType: string
    compression?: string
    throttle_rate?: number
    queue_size?: number
    latch?: boolean
    queue_length?: number
  }

  export class Topic {
    constructor(options: TopicOptions)
    
    subscribe(callback: (message: any) => void): void
    unsubscribe(callback?: (message: any) => void): void
    advertise(): void
    unadvertise(): void
    publish(message: any): void
  }

  export interface ServiceOptions {
    ros: Ros
    name: string
    serviceType: string
  }

  export class Service {
    constructor(options: ServiceOptions)
    
    callService(
      request: ServiceRequest,
      callback: (result: any) => void,
      failedCallback?: (error: any) => void
    ): void
  }

  export class ServiceRequest {
    constructor(values?: any)
  }

  export class ServiceResponse {
    constructor(values?: any)
  }

  export interface ParamOptions {
    ros: Ros
    name: string
  }

  export class Param {
    constructor(options: ParamOptions)
    
    get(callback: (value: any) => void): void
    set(value: any, callback?: () => void): void
    delete(callback?: () => void): void
  }

  export default Ros
}