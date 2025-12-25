import React from 'react';
import { useBaseUrl } from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

export default function RoboticsLayout(props) {
  const { siteConfig } = useDocusaurusContext();
  const { children, title, description } = props;

  return (
    <Layout title={title} description={description}>
      <main>{children}</main>
    </Layout>
  );
}