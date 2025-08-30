import React, { useEffect, useState } from 'react'
import { rosClient, prettyMeters, prettyDeg } from '../lib/ros.js'

// Expects geometry_msgs/PoseArray on topic (adjust if using a different ArUco message)
function yawFromQuat(q){
  // simple yaw from quaternion (z,yaw) approximation
  const { x=0,y=0,z=0,w=1 } = q || {}
  const siny_cosp = 2*(w*z + x*y)
  const cosy_cosp = 1 - 2*(y*y + z*z)
  return Math.atan2(siny_cosp, cosy_cosp)
}

function dist(p){ if (!p) return null; const {x=0,y=0,z=0} = p; return Math.sqrt(x*x+y*y+z*z) }

export default function ArucoPanel({ topic }){
  const [poses, setPoses] = useState([])
  useEffect(() => {
    if (!rosClient.ros) return
    const sub = rosClient.makeTopic({ name: topic, messageType: 'geometry_msgs/PoseArray', queue_size: 1, throttle_rate: 150 })
    sub.subscribe((msg) => {
      setPoses(msg.poses || [])
    })
    return () => sub.unsubscribe()
  }, [topic, rosClient.connected])

  return (
    <div style={{display:'grid', gridTemplateColumns:'repeat(2, minmax(0, 1fr))', gap:8}}>
      {poses.length === 0 && <div className="small">No markers yet…</div>}
      {poses.map((p, i) => (
        <div key={i} className="card" style={{padding:'8px'}}>
          <div className="small">Marker #{i+1}</div>
          <div>Distance: <b>{prettyMeters(dist(p.position))}</b></div>
          <div>Yaw: <b>{prettyDeg(yawFromQuat(p.orientation))}</b></div>
          <div className="small">pos: [{p.position?.x?.toFixed(2)||0}, {p.position?.y?.toFixed(2)||0}, {p.position?.z?.toFixed(2)||0}]</div>
        </div>
      ))}
    </div>
  )
}
