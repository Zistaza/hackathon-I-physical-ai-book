// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro', // existing intro.md
    {
      type: 'category',
      label: 'Module 1: ROS 2 for Humanoid Robotics',
      items: [
        'chapters/intro-ros2',
        'chapters/nodes-topics',
        'chapters/services-actions',
        'chapters/urdf-robot-description',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'modules/digital-twin/chapter-1-digital-twins-in-physical-ai',
        'modules/digital-twin/chapter-2-physics-simulation-with-gazebo',
        'modules/digital-twin/chapter-3-high-fidelity-environments-with-unity',
        'modules/digital-twin/chapter-4-sensor-simulation-for-humanoid-robots',
      ],
    },
  ],
};

module.exports = sidebars;
