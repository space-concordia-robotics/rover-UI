import ROSLIB from 'roslib'

// Singleton ROS connection helper
class RosClient {
  constructor() {
    const defaultUrl = (import.meta.env.VITE_ROSBRIDGE_URL) || `ws://${window.location.hostname}:9090`
    this.url = defaultUrl
    this.ros = null
    this.connected = false
    this.listeners = new Set()
  }
  connect(url) {
    this.url = url || this.url
    if (this.ros) { try { this.ros.close() } catch(e){} }
    if (!this.url || !this.url.startsWith('ws')) {
      // avoid creating WebSocket with malformed URL
      this.ros = null
      this.connected = false
      this._emit()
      return
    }
    try {
      this.ros = new ROSLIB.Ros({ url: this.url })
    } catch (e) {
      // invalid URL or other setup error
      console.error('ROS connection failed', e)
      this.ros = null
      this.connected = false
      this._emit()
      return
    }
    this.ros.on('connection', () => {
      this.connected = true
      this._emit()
    })
    this.ros.on('close', () => {
      this.connected = false
      this._emit()
      // try automatic reconnect
      setTimeout(() => this.connect(this.url), 1500)
    })
    this.ros.on('error', () => {
      this.connected = false
      this._emit()
    })
  }
  on(cb){ this.listeners.add(cb); cb(this); return () => this.listeners.delete(cb) }
  _emit(){ for (const cb of this.listeners) cb(this) }

  makeTopic({name, messageType, throttle_rate=0, queue_size=10}){
    return new ROSLIB.Topic({
      ros: this.ros,
      name, messageType, throttle_rate, queue_size
    })
  }

  callService(name, serviceType, request) {
    const srv = new ROSLIB.Service({ ros: this.ros, name, serviceType })
    return new Promise((resolve, reject) => {
      srv.callService(new ROSLIB.ServiceRequest(request||{}), resolve, reject)
    })
  }
}

export const rosClient = new RosClient()
export function prettyMeters(v){ return v==null?'-':`${v.toFixed(2)} m`}
export function prettyDeg(v){ return v==null?'-':`${v.toFixed(1)}°`}
export function prettyLL(lat, lon){
  if (lat==null || lon==null) return '-'
  return `${lat.toFixed(6)}, ${lon.toFixed(6)}`
}
