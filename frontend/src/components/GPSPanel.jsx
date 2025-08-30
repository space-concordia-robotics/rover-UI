import React, { useEffect, useState } from 'react'
import { rosClient, prettyLL } from '../lib/ros.js'

export default function GPSPanel({ topic }){
  const [fix, setFix] = useState(null)
  useEffect(() => {
    if (!rosClient.ros) return
    const sub = rosClient.makeTopic({ name: topic, messageType: 'sensor_msgs/NavSatFix', queue_size: 1, throttle_rate: 250 })
    sub.subscribe(setFix)
    return () => sub.unsubscribe()
  }, [topic, rosClient.connected])

  const lat = fix?.latitude, lon = fix?.longitude, alt = fix?.altitude
  return (
    <div className="row" style={{gap:24}}>
      <div><div className="small">Lat/Lon</div><div style={{fontSize:20}}>{prettyLL(lat, lon)}</div></div>
      <div><div className="small">Altitude</div><div style={{fontSize:20}}>{alt!=null ? `${alt.toFixed(1)} m` : '-'}</div></div>
      <div><div className="small">Status</div><div style={{fontSize:20}}>{fix ? fix.status?.status : '-'}</div></div>
    </div>
  )
}
