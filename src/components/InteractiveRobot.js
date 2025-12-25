import React, { useState, useEffect } from 'react';
import styles from './InteractiveRobot.module.css';

const InteractiveRobot = () => {
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });
  const [isHovering, setIsHovering] = useState(false);

  useEffect(() => {
    const handleMouseMove = (e) => {
      setMousePosition({ x: e.clientX, y: e.clientY });
    };

    window.addEventListener('mousemove', handleMouseMove);

    return () => {
      window.removeEventListener('mousemove', handleMouseMove);
    };
  }, []);

  const handleMouseEnter = () => {
    setIsHovering(true);
  };

  const handleMouseLeave = () => {
    setIsHovering(false);
  };

  return (
    <div
      className={styles.robotContainer}
      onMouseEnter={handleMouseEnter}
      onMouseLeave={handleMouseLeave}
    >
      <svg
        viewBox="0 0 200 200"
        className={styles.robotSvg}
        style={{
          transform: isHovering
            ? `translate(${(mousePosition.x - window.innerWidth / 2) * 0.02}px, ${(mousePosition.y - window.innerHeight / 2) * 0.02}px)`
            : 'translate(0px, 0px)',
          transition: isHovering ? 'transform 0.1s ease' : 'transform 0.3s ease'
        }}
      >
        {/* Robot Body */}
        <rect
          x="60"
          y="80"
          width="80"
          height="60"
          rx="10"
          fill="none"
          stroke="#00ff88"
          strokeWidth="3"
          className={styles.robotBody}
        />

        {/* Robot Head */}
        <circle
          cx="100"
          cy="60"
          r="25"
          fill="none"
          stroke="#00ff88"
          strokeWidth="3"
          className={styles.robotHead}
        />

        {/* Robot Eyes */}
        <circle
          cx="90"
          cy="55"
          r="4"
          fill={isHovering ? "#ff0088" : "#00ff88"}
          className={styles.robotEye}
        />
        <circle
          cx="110"
          cy="55"
          r="4"
          fill={isHovering ? "#ff0088" : "#00ff88"}
          className={styles.robotEye}
        />

        {/* Robot Mouth/Scanner */}
        <rect
          x="90"
          y="65"
          width="20"
          height="3"
          rx="1.5"
          fill={isHovering ? "#ff0088" : "#00ff88"}
          className={styles.robotMouth}
        />

        {/* Robot Arms */}
        <line
          x1="55"
          y1="90"
          x2="35"
          y2="110"
          stroke="#00ff88"
          strokeWidth="3"
          className={styles.robotArm}
        />
        <line
          x1="145"
          y1="90"
          x2="165"
          y2="110"
          stroke="#00ff88"
          strokeWidth="3"
          className={styles.robotArm}
        />

        {/* Robot Legs */}
        <line
          x1="80"
          y1="140"
          x2="80"
          y2="160"
          stroke="#00ff88"
          strokeWidth="3"
          className={styles.robotLeg}
        />
        <line
          x1="120"
          y1="140"
          x2="120"
          y2="160"
          stroke="#00ff88"
          strokeWidth="3"
          className={styles.robotLeg}
        />

        {/* Robot Antenna */}
        <line
          x1="100"
          y1="35"
          x2="100"
          y2="20"
          stroke="#00ff88"
          strokeWidth="2"
          className={styles.robotAntenna}
        />
        <circle
          cx="100"
          cy="15"
          r="5"
          fill={isHovering ? "#ff0088" : "#00ff88"}
          className={styles.robotAntennaTip}
        />

        {/* Sci-fi details */}
        <circle
          cx="100"
          cy="100"
          r="8"
          fill="none"
          stroke="#0088ff"
          strokeWidth="2"
          className={styles.robotCore}
        />
        <circle
          cx="100"
          cy="100"
          r="4"
          fill={isHovering ? "#ff0088" : "#0088ff"}
          className={styles.robotCoreLight}
        />
      </svg>

      {/* Sci-fi glow effect */}
      <div className={`${styles.robotGlow} ${isHovering ? styles.robotGlowActive : ''}`}></div>
    </div>
  );
};

export default InteractiveRobot;