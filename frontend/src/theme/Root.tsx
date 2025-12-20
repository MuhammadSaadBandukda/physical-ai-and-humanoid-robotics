import React from 'react';
import type RootType from '@theme/Root';
import { HardwareProvider } from '@site/src/context/HardwareContext';
import { LanguageProvider } from '@site/src/context/LanguageContext';
import { useUserProfile } from '@site/src/hooks/useUserProfile';

const RootContent = ({ children }: { children: React.ReactNode }) => {
  // Trigger the profile fetch when the app mounts
  useUserProfile();

  return <>{children}</>;
};

// Default implementation of Root from Docusaurus classic theme
// We just wrap the children with our HardwareProvider and LanguageProvider
const Root: typeof RootType = ({ children }) => (
  <HardwareProvider>
    <LanguageProvider>
      <RootContent>{children}</RootContent>
    </LanguageProvider>
  </HardwareProvider>
);

export default Root;


