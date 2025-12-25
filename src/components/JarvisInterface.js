import React, { useState, useEffect, useRef } from 'react';
import styles from './JarvisInterface.module.css';

const JarvisInterface = () => {
  const [isScanning, setIsScanning] = useState(false);
  const [activeModules, setActiveModules] = useState([]);
  const [systemStatus, setSystemStatus] = useState('STANDBY');
  const [scanProgress, setScanProgress] = useState(0);
  const containerRef = useRef(null);

  useEffect(() => {
    // Simulate system initialization
    const initTimer = setTimeout(() => {
      setSystemStatus('ONLINE');
      setActiveModules(['ROS_NERVOUS', 'DIGITAL_TWIN', 'ISAAC_AI', 'VLA_INTEGRATION']);
    }, 1000);

    // Simulate scanning animation
    const scanInterval = setInterval(() => {
      if (scanProgress < 100) {
        setScanProgress(prev => Math.min(prev + 2, 100));
      } else {
        setScanProgress(0);
      }
    }, 100);

    return () => {
      clearTimeout(initTimer);
      clearInterval(scanInterval);
    };
  }, [scanProgress]);

  const handleMouseEnter = () => {
    setIsScanning(true);
    setSystemStatus('SCANNING');
  };

  const handleMouseLeave = () => {
    setIsScanning(false);
    setSystemStatus('ONLINE');
  };

  return (
    <div
      className={styles.jarvisContainer}
      ref={containerRef}
      onMouseEnter={handleMouseEnter}
      onMouseLeave={handleMouseLeave}
    >
      {/* Main holographic display */}
      <div className={styles.holographicDisplay}>
        {/* Outer ring */}
        <div className={styles.outerRing}>
          <div className={styles.ringElement}></div>
          <div className={styles.ringElement}></div>
          <div className={styles.ringElement}></div>
        </div>

        {/* Inner data visualization */}
        <div className={styles.innerDisplay}>
          {/* Central core */}
          <div className={styles.core}>
            <div className={styles.coreGlow}></div>
            <div className={styles.corePulse}></div>
          </div>

          {/* Data streams */}
          <div className={styles.dataStreams}>
            {activeModules.map((module, index) => (
              <div
                key={module}
                className={`${styles.dataStream} ${styles[`dataStream${index}`]}`}
              >
                <div className={styles.dataNode}></div>
                <div className={styles.dataNode}></div>
                <div className={styles.dataNode}></div>
              </div>
            ))}
          </div>

          {/* Scanning beam */}
          <div className={`${styles.scanBeam} ${isScanning ? styles.scanBeamActive : ''}`}>
            <div className={styles.scanLine}></div>
            <div className={styles.scanLine}></div>
            <div className={styles.scanLine}></div>
          </div>

          {/* Status indicators */}
          <div className={styles.statusIndicators}>
            <div className={`${styles.statusLight} ${styles.statusGreen}`}></div>
            <div className={`${styles.statusLight} ${styles.statusBlue}`}></div>
            <div className={`${styles.statusLight} ${styles.statusCyan}`}></div>
          </div>
        </div>

        {/* Progress ring */}
        <div className={styles.progressRing}>
          <svg className={styles.progressSvg} viewBox="0 0 100 100">
            <circle
              className={styles.progressBackground}
              cx="50"
              cy="50"
              r="45"
              fill="none"
              stroke="rgba(0, 195, 255, 0.1)"
              strokeWidth="2"
            />
            <circle
              className={styles.progressCircle}
              cx="50"
              cy="50"
              r="45"
              fill="none"
              stroke="#00c3ff"
              strokeWidth="2"
              strokeDasharray={`${scanProgress * 2.83} 283`}
              transform="rotate(-90 50 50)"
            />
          </svg>
        </div>
      </div>

      {/* System status overlay */}
      <div className={styles.systemStatus}>
        <div className={styles.statusText}>{systemStatus}</div>
        <div className={styles.statusTextDetail}>
          {systemStatus === 'SCANNING' ? 'Analyzing environment...' :
           systemStatus === 'ONLINE' ? 'All systems operational' :
           'Initializing...'}
        </div>
      </div>

      {/* Floating particles for sci-fi effect */}
      <div className={styles.particles}>
        {[...Array(12)].map((_, i) => (
          <div key={i} className={`${styles.particle} ${styles[`particle${i}`]}`}></div>
        ))}
      </div>
    </div>
  );
};

export default JarvisInterface;