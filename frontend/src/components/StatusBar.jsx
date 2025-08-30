import React, { useEffect, useState } from 'react'
import { rosClient } from '../lib/ros.js'

export default function StatusBar({ cfg, onCfg }){
  const [connected, setConnected] = useState(false)
  useEffect(() => rosClient.on(() => setConnected(rosClient.connected)), [])

  return (
    <div className="card" style={{margin:'12px'}}>
      <div className="row" style={{justifyContent:'space-between'}}>
        <div className="row">
          <span className="pill">{ connected ? 'ROS Connected' : 'ROS Disconnected' }</span>
          <span className="status" style={{marginLeft:8}}>
            {connected ? <span className="ok">✓ {rosClient.url}</span> : <span className="bad">• trying {rosClient.url}</span>}
          </span>
        </div>
        <div className="row small">
          <label>rosbridge:</label>
          <input style={{minWidth:280}} value={cfg.rosbridgeUrl} onChange={e=>onCfg({...cfg, rosbridgeUrl:e.target.value})} />
          <label>web_video base:</label>
          <input style={{minWidth:280}} value={cfg.webVideoBase} onChange={e=>onCfg({...cfg, webVideoBase:e.target.value})} />
        </div>
      </div>
    </div>
  )
}
