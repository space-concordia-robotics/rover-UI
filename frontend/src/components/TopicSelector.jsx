import React from 'react'

export default function TopicSelector({ cfg, onCfg }){
  const t = cfg.topics
  function upd(key, val){ onCfg({ ...cfg, topics: { ...cfg.topics, [key]: val } }) }
  return (
    <div className="row" style={{gap:12}}>
      <div>
        <label>Camera</label><br/>
        <input value={t.camera} onChange={e=>upd('camera', e.target.value)} />
      </div>
      <div>
        <label>Map (OccupancyGrid)</label><br/>
        <input value={t.map} onChange={e=>upd('map', e.target.value)} />
      </div>
      <div>
        <label>GPS (NavSatFix)</label><br/>
        <input value={t.gps} onChange={e=>upd('gps', e.target.value)} />
      </div>
      <div>
        <label>LaserScan</label><br/>
        <input value={t.scan} onChange={e=>upd('scan', e.target.value)} />
      </div>
      <div>
        <label>ArUco PoseArray</label><br/>
        <input value={t.aruco_poses} onChange={e=>upd('aruco_poses', e.target.value)} />
      </div>
    </div>
  )
}
