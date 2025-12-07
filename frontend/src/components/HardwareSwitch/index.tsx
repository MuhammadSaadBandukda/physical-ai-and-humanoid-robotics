import React from 'react';
import { useHardware, HardwareMode } from '@site/src/context/HardwareContext';
import styles from './styles.module.css'; // Assuming we'll create a CSS module

const HardwareSwitch: React.FC = () => {
  const { mode, setMode } = useHardware();

  const toggleMode = () => {
    setMode(mode === 'sim' ? 'physical' : 'sim');
  };

  const buttonStyle: React.CSSProperties = {
    backgroundColor: mode === 'sim' ? '#28a745' : '#ffc107', // Green for Sim, Orange for Physical
    color: 'white',
    border: 'none',
    padding: '10px 20px',
    borderRadius: '5px',
    cursor: 'pointer',
    fontSize: '1em',
    fontWeight: 'bold',
    margin: '10px 0',
  };

  return (
    <div className={styles.hardwareSwitchContainer}>
      <p>Current Mode: <span style={{ fontWeight: 'bold' }}>{mode.toUpperCase()}</span></p>
      <button onClick={toggleMode} style={buttonStyle}>
        Switch to {mode === 'sim' ? 'Physical' : 'Sim'}
      </button>
      <p style={{ fontSize: '0.8em', color: '#666' }}>
        ({mode === 'sim' ? 'Digital Twin (RTX)' : 'Physical AI (Jetson)'})
      </p>
    </div>
  );
};

export default HardwareSwitch;
