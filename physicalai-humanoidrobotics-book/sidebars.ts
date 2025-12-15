import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

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
  // Manual sidebar for Physical AI & Humanoid Robotics textbook
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'preface',
      label: 'Preface: Welcome to the AI-Native Era',
    },
    {
      type: 'html',
      value: '<hr style="margin: 0.5rem 0; border: none; border-top: 1px solid var(--ifm-toc-border-color);" />',
      defaultStyle: true,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'module1/module-overview',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module1/chapter1-installation',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module1/chapter2-topics',
        // TODO: Uncomment as chapters are created
        // {
        //   type: 'html',
        //   value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        // },
        // 'module1/chapter3-python',
        // {
        //   type: 'html',
        //   value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        // },
        // 'module1/chapter4-services',
        // {
        //   type: 'html',
        //   value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        // },
        // 'module1/chapter5-urdf',
        // {
        //   type: 'html',
        //   value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        // },
        // 'module1/integration-project',
      ],
    },
    {
      type: 'html',
      value: '<hr style="margin: 0.5rem 0; border: none; border-top: 1px solid var(--ifm-toc-border-color);" />',
      defaultStyle: true,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        'module2/module-overview',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module2/chapter1-gazebo-physics',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module2/chapter2-unity-rendering',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module2/chapter3-sensor-simulation',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module2/integration-project',
      ],
    },
    {
      type: 'html',
      value: '<hr style="margin: 0.5rem 0; border: none; border-top: 1px solid var(--ifm-toc-border-color);" />',
      defaultStyle: true,
    },
    {
      type: 'category',
      label: 'Module 3: Isaac ROS for Humanoid Robotics',
      collapsed: false,
      items: [
        'module3/module-overview',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module3/chapter1-isaac-sim',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module3/chapter2-vslam',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module3/chapter3-nav2',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module3/chapter4-integration',
        {
          type: 'html',
          value: '<hr style="margin: 0.3rem 0; border: none; border-top: 1px dashed var(--ifm-toc-border-color); opacity: 0.5;" />',
        },
        'module3/summary',
      ],
    },
    {
      type: 'html',
      value: '<hr style="margin: 0.5rem 0; border: none; border-top: 1px solid var(--ifm-toc-border-color);" />',
      defaultStyle: true,
    },
  ],
};

export default sidebars;
