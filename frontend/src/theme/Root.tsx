import React from 'react';
import type RootType from '@theme/Root';
import { HardwareProvider } from '@site/src/context/HardwareContext';
import { useUserProfile } from '@site/src/hooks/useUserProfile';

const RootContent = ({ children }: { children: React.ReactNode }) => {
  // Trigger the profile fetch when the app mounts
  // We don't necessarily need to render the data here, just ensure the side effect runs
  // as per Task T015 requirements ("Console logs user profile data on page load").
  useUserProfile();

  return <>{children}</>;
};

// Default implementation of Root from Docusaurus classic theme
// We just wrap the children with our HardwareProvider
const Root: typeof RootType = ({ children }) => (
  <HardwareProvider>
    <RootContent>{children}</RootContent>
  </HardwareProvider>
);

export default Root;

