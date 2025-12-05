import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Main course sidebar
  courseSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-01-ros2/overview',
        'module-01-ros2/chapter-01-physical-ai',
        'module-01-ros2/chapter-02-ros2-fundamentals',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twins',
      items: [
        'module-02-digital-twins/overview',
        'module-02-digital-twins/chapter-03-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Sim',
      items: [
        'module-03-isaac-sim/overview',
        'module-03-isaac-sim/chapter-04-ai-robot-brain',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Systems',
      items: [
        'module-04-vla-systems/overview',
        'module-04-vla-systems/chapter-05-vla',
      ],
    },
    {
      type: 'category',
      label: 'Course Structure',
      items: [
        'course-structure/weekly-schedule',
        'course-structure/assessments',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Architecture',
      items: [
        'hardware-architecture/overview',
      ],
    },
  ],

  // Reference sidebar
  referenceSidebar: [
    {
      type: 'doc',
      id: 'reference/glossary',
      label: 'Glossary',
    },
    {
      type: 'doc',
      id: 'reference/bibliography',
      label: 'Bibliography',
    },
  ],
};

export default sidebars;
