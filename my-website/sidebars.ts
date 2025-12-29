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
    {
     type: 'category',
     label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
     items: [
    'module3-ai-robot-brain/intro-isaac-sim',
    'module3-ai-robot-brain/isaac-ros-vslam',
    'module3-ai-robot-brain/nav2-path-planning',
    'module3-ai-robot-brain/integration-deployment',
     ],
    },
    {
    type: 'category',
    label: 'Module 4: Vision-Language-Action (VLA) in Physical AI & Humanoid Robotics',
    items: [
    'module4-vla-physical-ai/intro-vla',
    'module4-vla-physical-ai/voice-to-action',
    'module4-vla-physical-ai/cognitive-planning-llms',
    'module4-vla-physical-ai/capstone-autonomous-humanoid',
     ],
     },
     {
     type: 'category',
     label: 'Hardware Setup',
     items: ['hardware-requirements'],
     },



  ],
};

module.exports = sidebars;
