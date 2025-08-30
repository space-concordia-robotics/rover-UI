import React, { useEffect, useMemo, useRef, useState } from 'react'
import { GridStack } from 'gridstack'
import 'gridstack/dist/gridstack.min.css'
import { createRoot } from 'react-dom/client'
import { rosClient } from './lib/ros.js'
import StatusBar from './components/StatusBar.jsx'
import TopicSelector from './components/TopicSelector.jsx'
import CameraPanel from './components/CameraPanel.jsx'
import MapPanel from './components/MapPanel.jsx'
import GPSPanel from './components/GPSPanel.jsx'
import LaserScanPanel from './components/LaserScanPanel.jsx'
import ArucoPanel from './components/ArucoPanel.jsx'
import TerminalPanel from './components/TerminalPanel.jsx'

const defaultConfig = {
  rosbridgeUrl: (import.meta.env.VITE_ROSBRIDGE_URL) || `ws://${window.location.hostname}:9090`,
  webVideoBase: (import.meta.env.VITE_WEBCAM_STREAM_BASE) || `http://${window.location.hostname}:8080/stream`,
  topics: {
    camera: '/camera/image_raw',
    map: '/map', // nav_msgs/OccupancyGrid
    gps: '/fix', // sensor_msgs/NavSatFix
    scan: '/scan', // sensor_msgs/LaserScan
    aruco_poses: '/aruco/poses' // geometry_msgs/PoseArray (adjust as needed)
  }
}

const defaultLayout = [
  { type: 'camera', x: 0, y: 0, w: 6, h: 4 },
  { type: 'map', x: 6, y: 0, w: 4, h: 2 },
  { type: 'gps', x: 0, y: 4, w: 2, h: 2 },
  { type: 'scan', x: 6, y: 4, w: 6, h: 4 },
  { type: 'aruco_poses', x: 0, y: 8, w: 6, h: 4 },
  { type: 'topics', x: 6, y: 8, w: 6, h: 4 },
  { type: 'terminal', x: 0, y: 12, w: 12, h: 4 }
]

function useLocalConfig() {
  const [cfg, setCfg] = useState(() => {
    const saved = localStorage.getItem('rover_ui_config')
    return saved ? JSON.parse(saved) : defaultConfig
  })
  useEffect(() => { localStorage.setItem('rover_ui_config', JSON.stringify(cfg)) }, [cfg])
  return [cfg, setCfg]
}

export default function App(){
  const [cfg, setCfg] = useLocalConfig()
  const gridContainer = useRef(null)
  const grid = useRef(null)
  const roots = useRef(new Map())
  const fileInputRef = useRef(null)
  const [newType, setNewType] = useState('camera')
  const [newTopic, setNewTopic] = useState('')
  const [autoArrange, setAutoArrange] = useState(true)
  const autoArrangeRef = useRef(autoArrange)

  useEffect(() => { autoArrangeRef.current = autoArrange }, [autoArrange])
  useEffect(() => {
    if (!grid.current) return
    grid.current.float(!autoArrange)
    if (autoArrange) grid.current.compact()
  }, [autoArrange])
  const compactIfNeeded = () => {
    if (autoArrangeRef.current) grid.current?.compact()
  }

  useEffect(() => { rosClient.connect(cfg.rosbridgeUrl) }, [cfg.rosbridgeUrl])

  useEffect(() => {
    grid.current = GridStack.init({ cellHeight: 150 }, gridContainer.current)
    const doCompact = () => compactIfNeeded()
    grid.current.on('dragstop', doCompact)
    grid.current.on('resizestop', doCompact)
    grid.current.on('added removed', doCompact)
    applyLayout(defaultLayout)
    return () => grid.current?.destroy()
  }, [])

  const panelDefs = useMemo(() => ({
    camera: {
      label: 'Camera',
      topicKey: 'camera',
      render: (topic) => <CameraPanel baseUrl={cfg.webVideoBase} topic={topic} />
    },
    map: {
      label: 'Map (OccupancyGrid)',
      topicKey: 'map',
      render: (topic) => <MapPanel topic={topic} />
    },
    gps: {
      label: 'GPS',
      topicKey: 'gps',
      render: (topic) => <GPSPanel topic={topic} />
    },
    scan: {
      label: 'LiDAR / LaserScan',
      topicKey: 'scan',
      render: (topic) => <LaserScanPanel topic={topic} />
    },
    aruco_poses: {
      label: 'ArUco Poses',
      topicKey: 'aruco_poses',
      render: (topic) => <ArucoPanel topic={topic} />
    },
    terminal: {
      label: 'Terminal',
      render: () => <TerminalPanel />
    },
    topics: {
      label: 'Topics',
      render: () => <TopicSelector cfg={cfg} onCfg={setCfg} />
    }
  }), [cfg])

  const selectableDefs = useMemo(
    () => Object.entries(panelDefs).filter(([key]) => key !== 'topics'),
    [panelDefs]
  )

  const removeWidget = (e) => {
    const item = e.target.closest('.grid-stack-item')
    const data = roots.current.get(item)
    if (data) {
      data.root.unmount()
      roots.current.delete(item)
    }
    grid.current?.removeWidget(item)
    compactIfNeeded()
  }

  const renderPanel = (root, type, topic) => {
    const def = panelDefs[type]
    if (!def) return
    root.render(<>
      <h3 className="title">{def.label}</h3>
      {def.render(topic)}
    </>)
  }
  const createWidget = ({ type, override = null, x, y, w = 6, h = 4 }) => {
    const el = document.createElement('div')
    el.className = 'grid-stack-item'
    el.innerHTML =
      '<div class="grid-stack-item-content card"><button class="remove btn small">Remove</button><div class="content"></div></div>'
    grid.current?.addWidget(el, { x, y, w, h })
    grid.current?.update(el, { x, y, w, h })
    const content = el.querySelector('.content')
    const root = createRoot(content)
    roots.current.set(el, { root, type, override })
    const def = panelDefs[type]
    const topic = override || (def?.topicKey ? cfg.topics[def.topicKey] : null)
    renderPanel(root, type, topic)
    el.querySelector('.remove').addEventListener('click', removeWidget)
  }

  const addWidget = () => {
    const def = panelDefs[newType]
    if (!def) return
    const override = newTopic.trim() || null
    createWidget({ type: newType, override })
    compactIfNeeded()
    setNewTopic('')
  }

  const applyLayout = (layout) => {
    if (!grid.current) return
    roots.current.forEach(({ root }, el) => root.unmount())
    roots.current.clear()
    grid.current.removeAll()
    layout.forEach(createWidget)
    compactIfNeeded()
  }

  const saveLayoutToFile = () => {
    if (!grid.current) return
    const layout = []
    roots.current.forEach(({ type, override }, el) => {
      const n = el.gridstackNode
      layout.push({ x: n.x, y: n.y, w: n.w, h: n.h, type, override })
    })
    const blob = new Blob([JSON.stringify(layout, null, 2)], { type: 'application/json' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = 'layout.json'
    a.click()
    URL.revokeObjectURL(url)
  }

  const handleLayoutFile = (e) => {
    const file = e.target.files?.[0]
    if (!file) return
    const reader = new FileReader()
    reader.onload = () => {
      try {
        const layout = JSON.parse(reader.result)
        applyLayout(layout)
      } catch (err) {
        console.error('Error parsing layout file', err)
      }
    }
    reader.readAsText(file)
  }

  useEffect(() => {
    roots.current.forEach(({ root, type, override }) => {
      const def = panelDefs[type]
      if (def) {
        const topic = override || (def.topicKey ? cfg.topics[def.topicKey] : null)
        root.render(<>
          <h3 className="title">{def.label}</h3>
          {def.render(topic)}
        </>)
      }
    })
  }, [cfg])

  return (
    <div>
      <StatusBar cfg={cfg} onCfg={setCfg} />
      <div className="row" style={{ margin: '12px' }}>
        <select value={newType} onChange={(e) => setNewType(e.target.value)}>
          {selectableDefs.map(([key, def]) => (
            <option key={key} value={key}>
              {def.label}
            </option>
          ))}
        </select>
        <input
          placeholder="topic (optional)"
          value={newTopic}
          onChange={(e) => setNewTopic(e.target.value)}
        />
        <button onClick={addWidget}>Add Panel</button>
        <label style={{ marginLeft: '12px' }}>
          <input
            type="checkbox"
            checked={autoArrange}
            onChange={(e) => setAutoArrange(e.target.checked)}
          />{' '}
          Auto arrange
        </label>
        <button style={{ marginLeft: '12px' }} onClick={saveLayoutToFile}>
          Save Layout
        </button>
        <button onClick={() => fileInputRef.current?.click()}>Load Layout</button>
        <input
          type="file"
          accept="application/json"
          style={{ display: 'none' }}
          ref={fileInputRef}
          onChange={handleLayoutFile}
        />
      </div>
      <div className="grid-stack" ref={gridContainer}></div>
    </div>
  )
}
