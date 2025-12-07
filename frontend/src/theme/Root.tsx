import React from 'react';
import type RootType from '@theme/Root';
import { HardwareProvider } from '@site/src/context/HardwareContext';

// Default implementation of Root from Docusaurus classic theme
// We just wrap the children with our HardwareProvider
const Root: typeof RootType = ({ children }) => (
  <HardwareProvider>
    {children}
  </HardwareProvider>
);

export default Root;
