import React, { useState, useEffect, useRef } from 'react';

const WS_BASE = `ws://${window.location.host}`;

function useWebSocket(path) {
  const [data, setData] = useState(null);
  const wsRef = useRef(null);

  useEffect(() => {
    const connect = () => {
      const ws = new WebSocket(`${WS_BASE}${path}`);
      wsRef.current = ws;
      ws.onmessage = (e) => setData(JSON.parse(e.data));
      ws.onclose = () => setTimeout(connect, 2000);
      ws.onerror = () => ws.close();
    };
    connect();
    return () => wsRef.current?.close();
  }, [path]);

  return data;
}

function StatusBadge({ health }) {
  const colors = { OK: '#22c55e', WARN: '#f59e0b', ERROR: '#ef4444' };
  return (
    <span style={{
      background: colors[health] || '#6b7280',
      color: 'white', padding: '2px 8px', borderRadius: 4, fontSize: 12
    }}>
      {health || 'UNKNOWN'}
    </span>
  );
}

function Panel({ title, children }) {
  return (
    <div style={{ border: '1px solid #e5e7eb', borderRadius: 8, padding: 16, margin: 8 }}>
      <h3 style={{ margin: '0 0 8px', fontSize: 14, color: '#374151' }}>{title}</h3>
      {children}
    </div>
  );
}

function CameraPanel({ cam }) {
  if (!cam) return <Panel title="Camera"><p>No data</p></Panel>;
  return (
    <Panel title="Camera">
      <p>Connected: {cam.connected ? 'Yes' : 'No'}</p>
      <p>Frame Delta: {cam.frameDeltaMs?.toFixed(1)} ms</p>
      <p>Depth FPS: {cam.depthFps?.toFixed(1)}</p>
      <p>IMU Active: {cam.imuActive ? 'Yes' : 'No'}</p>
    </Panel>
  );
}

function Px4Panel({ px4 }) {
  if (!px4) return <Panel title="PX4"><p>No data</p></Panel>;
  return (
    <Panel title="PX4">
      <p>Connected: {px4.connected ? 'Yes' : 'No'}</p>
      <p>Armed: {px4.armed ? 'Yes' : 'No'}</p>
      <p>Armable: {px4.armable ? 'Yes' : 'No'}</p>
      <p>Nav State: {px4.navStateLabel}</p>
      <p>Battery: {px4.batteryRemainingPct?.toFixed(0)}% ({px4.batteryVoltageV?.toFixed(1)}V)</p>
      <p>Heartbeat Age: {px4.heartbeatAgeMs} ms</p>
    </Panel>
  );
}

function JetsonPanel({ jetson }) {
  if (!jetson) return <Panel title="Jetson"><p>No data</p></Panel>;
  return (
    <Panel title="Jetson">
      <p>GPU: {jetson.gpuUsagePct?.toFixed(1)}%</p>
      <p>RAM: {jetson.ramUsedMb}/{jetson.ramTotalMb} MB</p>
      <p>Temp CPU: {jetson.tempCpuC?.toFixed(1)}C</p>
      <p>Disk Free: {jetson.diskFreeGb?.toFixed(1)} GB</p>
      <p>Thermal Throttle: {jetson.isThermalThrottled ? 'YES' : 'No'}</p>
    </Panel>
  );
}

function AlertsPanel({ alerts }) {
  return (
    <Panel title="Active Alerts">
      {alerts && alerts.length > 0 ? (
        <ul style={{ margin: 0, paddingLeft: 16 }}>
          {alerts.map((a, i) => <li key={i} style={{ color: '#ef4444' }}>{a}</li>)}
        </ul>
      ) : (
        <p style={{ color: '#22c55e' }}>No active alerts</p>
      )}
    </Panel>
  );
}

function CommandPanel() {
  const [log, setLog] = useState([]);
  const ackData = useWebSocket('/ws/cmd_ack');

  useEffect(() => {
    if (ackData) {
      setLog(prev => [ackData, ...prev].slice(0, 50));
    }
  }, [ackData]);

  const sendCmd = async (cmdType, params = {}) => {
    try {
      const res = await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ cmd_type: cmdType, params, issued_by: 'dashboard' }),
      });
      const data = await res.json();
      if (data.error) alert(`Rejected: ${data.error}`);
    } catch (e) {
      alert(`Failed: ${e.message}`);
    }
  };

  return (
    <Panel title="Commands">
      <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap', marginBottom: 12 }}>
        <button onClick={() => sendCmd('arm')}>Arm</button>
        <button onClick={() => sendCmd('disarm')}>Disarm</button>
        <button onClick={() => sendCmd('estop')} style={{ background: '#ef4444', color: 'white' }}>
          E-STOP
        </button>
        <button onClick={() => sendCmd('cancel_goal')}>Cancel Goal</button>
      </div>
      <h4 style={{ margin: '8px 0 4px', fontSize: 12 }}>Command Log</h4>
      <div style={{ maxHeight: 200, overflow: 'auto', fontSize: 11 }}>
        {log.map((entry, i) => (
          <div key={i} style={{ borderBottom: '1px solid #f3f4f6', padding: '2px 0' }}>
            {entry.cmdId}: {entry.ackStatus} ({entry.roundTripMs}ms) - {entry.message}
          </div>
        ))}
      </div>
    </Panel>
  );
}

export default function App() {
  const health = useWebSocket('/ws/health');

  return (
    <div style={{ fontFamily: 'system-ui', maxWidth: 1000, margin: '0 auto', padding: 16 }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 12, marginBottom: 16 }}>
        <h1 style={{ fontSize: 20, margin: 0 }}>Rover Control Center</h1>
        <StatusBadge health={health?.overallHealth} />
        {health && <span style={{ fontSize: 12, color: '#6b7280' }}>
          SLAM: {health.slamLatencyMs?.toFixed(0)}ms | seq: {health.seq}
        </span>}
      </div>

      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))', gap: 0 }}>
        <CameraPanel cam={health?.camera} />
        <Px4Panel px4={health?.px4} />
        <JetsonPanel jetson={health?.jetson} />
        <AlertsPanel alerts={health?.activeAlerts} />
        <CommandPanel />
      </div>
    </div>
  );
}
