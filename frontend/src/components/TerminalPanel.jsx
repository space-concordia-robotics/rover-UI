import React, { useState } from 'react'
import { rosClient } from '../lib/ros.js'

export default function TerminalPanel(){
  const [cmd, setCmd] = useState('')
  const [log, setLog] = useState('')
  const [busy, setBusy] = useState(false)

  const run = async () => {
    const command = cmd.trim()
    if (!command || busy) return
    setBusy(true)
    setLog(prev => prev + `\n$ ${command}\n`)
    try {
      const resp = await rosClient.callService('/run_command', 'sc_msgs/RunCommand', { command })
      if (resp && resp.output != null) {
        setLog(prev => prev + resp.output + '\n')
      }
    } catch (err) {
      setLog(prev => prev + '[error] ' + err + '\n')
    }
    setBusy(false)
    setCmd('')
  }

  const handleKey = e => {
    if (e.key === 'Enter') {
      e.preventDefault()
      run()
    }
  }

  return (
    <div style={{display:'flex', flexDirection:'column', height:'100%'}}>
      <pre style={{flex:1, margin:0, background:'#000', color:'#0f0', padding:'8px', overflow:'auto'}}>{log || 'Ready.'}</pre>
      <div className="row">
        <input
          style={{flex:1, fontFamily:'monospace'}}
          value={cmd}
          onChange={e=>setCmd(e.target.value)}
          onKeyDown={handleKey}
          placeholder="command"
          disabled={busy}
        />
        <button onClick={run} disabled={busy}>Run</button>
      </div>
    </div>
  )
}
