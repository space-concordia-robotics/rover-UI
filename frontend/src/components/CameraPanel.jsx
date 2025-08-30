import React, { useMemo } from 'react'

export default function CameraPanel({ baseUrl, topic }){
  const src = useMemo(() => {
    // web_video_server query params: ?topic=/camera/image_raw&type=mjpeg&quality=80
    const u = new URL(baseUrl)
    u.searchParams.set('topic', topic)
    if (!u.searchParams.get('type')) u.searchParams.set('type','mjpeg')
    if (!u.searchParams.get('quality')) u.searchParams.set('quality','80')
    return u.toString()
  }, [baseUrl, topic])

  return (
    <div>
      <div className="small" style={{marginBottom:8}}>Source: <span className="kbd">{src}</span></div>
      <img className="cam" src={src} alt="camera stream" />
    </div>
  )
}
