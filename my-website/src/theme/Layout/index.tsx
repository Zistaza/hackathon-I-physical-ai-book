import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import type {Props} from '@theme/Layout';
import useIsBrowser from '@docusaurus/useIsBrowser';

// Import the RAG chatbot component
const RagChatbot = require('../../components/RagChatbot/index.js').default;

export default function Layout(props: Props): JSX.Element {
  const isBrowser = useIsBrowser();

  return (
    <>
      <OriginalLayout {...props} />
      {isBrowser && <RagChatbot />}
    </>
  );
}