import React, { useState } from 'react';
import { useLanguage } from '@site/src/context/LanguageContext';
import styles from './styles.module.css';
import Toast from '../Toast';

const TranslationSwitch: React.FC = () => {
  const { language, setLanguage } = useLanguage();
  const [showToast, setShowToast] = useState(false);

  const toggleLanguage = () => {
    if (language === 'EN') {
      // Intentional Fallback for Prototype: Show Error
      setShowToast(true);
      // We DO NOT actually switch state to 'UR' to avoid breaking the UI with missing content,
      // OR we switch it but acknowledge the content won't change.
      // Per task requirements: "Switch the visual state... Trigger a Toast".
      
      // Let's toggle it to show the visual change, but the content won't actually translate yet.
      setLanguage('UR');
    } else {
      setLanguage('EN');
    }
  };

  const buttonStyle: React.CSSProperties = {
    backgroundColor: language === 'EN' ? '#007bff' : '#6c757d', // Blue for English, Gray for Urdu
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
    <div className={styles.translationSwitchContainer}>
      {showToast && (
        <Toast
          message="Translation Service Unavailable: API Key missing or Backend offline."
          type="error"
          onClose={() => setShowToast(false)}
        />
      )}
      <p>Language: <span style={{ fontWeight: 'bold' }}>{language}</span></p>
      <button onClick={toggleLanguage} style={buttonStyle}>
        Switch to {language === 'EN' ? 'Urdu' : 'English'}
      </button>
      <p style={{ fontSize: '0.8em', color: '#666' }}>
        ({language === 'EN' ? 'Original' : 'AI Translated'})
      </p>
    </div>
  );
};

export default TranslationSwitch;
