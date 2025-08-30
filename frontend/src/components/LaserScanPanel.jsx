import React, { useEffect, useRef, useState } from 'react'
import { rosClient } from '../lib/ros.js'

function drawScan(canvas, msg){
  const ctx = canvas.getContext('2d')
  const W = canvas.width = canvas.clientWidth
  const H = canvas.height = Math.min(canvas.clientHeight || 400, 400)
  ctx.clearRect(0,0,W,H)
  ctx.save()
  ctx.translate(W/2, H*0.9)
  ctx.scale(1, -1)
  ctx.fillStyle = '#2a3da3'
  ctx.strokeStyle = '#8fa2ff'
  ctx.lineWidth = 1

  const scale = Math.min(W, H) * 0.4 // pixels per meter approx

  const { angle_min, angle_increment, ranges, range_min, range_max } = msg
  ctx.beginPath()
  for (let i=0; i<ranges.length; i++){
    const r = ranges[i]
    if (!isFinite(r) || r < range_min || r > range_max) continue
    const ang = angle_min + i * angle_increment
    const x = Math.cos(ang) * r * scale
    const y = Math.sin(ang) * r * scale
    ctx.moveTo(0,0); ctx.lineTo(x,y)
  }
  ctx.stroke()

  // robot footprint
  ctx.fillStyle = '#7bd0a3'
  ctx.beginPath()
  ctx.arc(0,0,6,0,Math.PI*2)
  ctx.fill()
  ctx.restore()
}

export default function LaserScanPanel({ topic }){
  const canvasRef = useRef(null)
  const [lastHdr, setLastHdr] = useState(null)

  useEffect(() => {
    if (!rosClient.ros) return
    const sub = rosClient.makeTopic({ name: topic, messageType: 'sensor_msgs/LaserScan', queue_size: 1, throttle_rate: 120 })
    sub.subscribe((msg) => {
      setLastHdr(msg.header)
      drawScan(canvasRef.current, msg)
    })
    return () => sub.unsubscribe()
  }, [topic, rosClient.connected])

  return (
    <div>
      <div className="small" style={{marginBottom:8}}>Frame: {lastHdr?.frame_id || '-'}</div>
      <canvas ref={canvasRef} style={{width:'100%', height:'360px'}} />
    </div>
  )
}
