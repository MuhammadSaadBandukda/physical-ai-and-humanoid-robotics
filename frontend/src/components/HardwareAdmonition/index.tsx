import React, { ReactNode } from 'react';
import Admonition from '@theme/Admonition'; // Docusaurus Admonition component
import { useHardware } from '@site/src/context/HardwareContext'; // Import useHardware

interface HardwareAdmonitionProps {
  type: 'note' | 'tip' | 'info' | 'warning' | 'danger'; // Standard Docusaurus admonition types
  title?: string;
  hardware: 'RTX' | 'Jetson' | 'All'; // New prop: specific hardware for this admonition
  children: ReactNode;
}

const HardwareAdmonition: React.FC<HardwareAdmonitionProps> = ({ type, title, hardware, children }) => {
  const { mode } = useHardware(); // Get current hardware mode from context

  // Determine if this admonition should be visible
  // 'RTX' maps to 'sim' mode, 'Jetson' maps to 'physical' mode
  const shouldShow =
    hardware === 'All' ||
    (hardware === 'RTX' && mode === 'sim') ||
    (hardware === 'Jetson' && mode === 'physical');

  if (!shouldShow) {
    return null; // Don't render the admonition if it's not for the current mode
  }

  return (
    <Admonition type={type} title={title}>
      {children}
    </Admonition>
  );
};

export default HardwareAdmonition;

