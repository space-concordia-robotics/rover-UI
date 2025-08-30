import React, { useEffect, useRef, useState } from 'react'
import { rosClient } from '../lib/ros.js'

function drawGrid(canvas, msg){
  const ctx = canvas.getContext('2d')
  const { width, height, data } = msg.info
  const grid = msg.data
  // create ImageData
  const im = ctx.createImageData(width, height)
  for (let i=0; i<width*height; i++){
    const v = grid[i] // -1 = unknown, 0..100
    let g
    if (v < 0) g = 30 // dark for unknown
    else g = 255 - Math.floor((v/100)*255)
    const idx = i*4
    im.data[idx+0] = g
    im.data[idx+1] = g
    im.data[idx+2] = g
    im.data[idx+3] = 255
  }
  // flip Y so (0,0) at bottom-left
  const tmp = document.createElement('canvas')
  tmp.width = width; tmp.height = height
  tmp.getContext('2d').putImageData(im, 0, 0)
  canvas.width = width; canvas.height = height
  ctx.save()
  ctx.scale(1, -1)
  ctx.drawImage(tmp, 0, -height)
  ctx.restore()
}

export default function MapPanel({ topic }){
  const canvasRef = useRef(null)
  const [lastInfo, setLastInfo] = useState(null)

  useEffect(() => {
    if (!rosClient.ros) return
    const sub = rosClient.makeTopic({ name: topic, messageType: 'nav_msgs/OccupancyGrid', queue_size: 1, throttle_rate: 200 })
    sub.subscribe((msg) => {
      setLastInfo(msg.info)
      drawGrid(canvasRef.current, msg)
    })
    return () => sub.unsubscribe()
  }, [topic, rosClient.connected])

  return (
    <div>
      <div className="small" style={{marginBottom:8}}>
        Res: {lastInfo ? lastInfo.resolution.toFixed(2) : '-'} m/px
        &nbsp;&nbsp; Size: {lastInfo ? `${lastInfo.width}×${lastInfo.height}` : '-'}
      </div>
      <canvas ref={canvasRef} />
    </div>
  )
}
