import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import styles from './404.module.css';

export default function NotFound() {
  return (
    <Layout title="Page Not Found">
      <main className="container margin-vert--xl">
        <div className={clsx('row', styles.notFoundRow)}>
          <div className="col col--6 col--offset-3">
            <div className={styles.notFoundContainer}>
              <h1 className={styles.notFoundTitle}>404</h1>
              <p className={styles.notFoundSubtitle}>
                Page Not Found
              </p>
              <p className={styles.notFoundDescription}>
                We couldn't find the page you were looking for.
              </p>
              <div className={styles.notFoundActions}>
                <Link to="/" className="button button--primary button--lg">
                  Go Home
                </Link>
                <Link to="/docs/intro" className="button button--secondary button--lg">
                  Documentation
                </Link>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}