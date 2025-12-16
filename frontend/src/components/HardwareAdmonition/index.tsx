import React, { ReactNode } from 'react';
import Admonition from '@theme/Admonition'; // Docusaurus Admonition component

interface HardwareAdmonitionProps {
  type: 'note' | 'tip' | 'info' | 'warning' | 'danger'; // Standard Docusaurus admonition types
  title?: string;
  children: ReactNode;
}

const HardwareAdmonition: React.FC<HardwareAdmonitionProps> = ({ type, title, children }) => {
  return (
    <Admonition type={type} title={title}>
      {children}
    </Admonition>
  );
};

export default HardwareAdmonition;
