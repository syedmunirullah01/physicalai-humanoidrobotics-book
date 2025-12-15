/**
 * ChapterLayoutExample - Demonstrates how to use ChapterPage component
 * This file shows the integration pattern for chapter content
 */

import React from 'react';
import { ChapterPage, type TOCItem, type NavChapter } from '@site/src/components';

/**
 * Example: How to wrap a chapter MDX file with ChapterPage
 *
 * In your .mdx file, you would do:
 *
 * ```mdx
 * import { ChapterPage } from '@site/src/components';
 *
 * <ChapterPage
 *   moduleNumber={1}
 *   moduleName="ROS 2 Fundamentals"
 *   chapterNumber={1}
 *   chapterTitle="ROS 2 Installation and Workspace Setup"
 *   bloomLevel="Understand"
 *   estimatedTime="1.5-2 hours"
 *   progress={0}
 *   tableOfContents={[...]}
 *   previousChapter={undefined}
 *   nextChapter={{
 *     title: "Nodes, Topics, and Publishers/Subscribers",
 *     link: "/docs/module1/chapter2-topics"
 *   }}
 * >
 *   [Your chapter content here - use markdown or MDX components]
 * </ChapterPage>
 * ```
 */

// Example TOC structure
const exampleTOC: TOCItem[] = [
  {
    id: 'ros2-fundamentals',
    title: 'ROS 2 Fundamentals',
    level: 1,
    children: [
      {
        id: 'what-is-ros',
        title: 'What is ROS 2?',
        level: 2,
      },
      {
        id: 'architecture-overview',
        title: 'Architecture Overview',
        level: 2,
      },
    ],
  },
  {
    id: 'installation-guide',
    title: 'Installation Guide',
    level: 1,
    children: [
      {
        id: 'ubuntu-installation',
        title: 'Ubuntu 22.04 LTS Installation',
        level: 2,
      },
      {
        id: 'wsl2-installation',
        title: 'Windows WSL2 Setup',
        level: 2,
      },
      {
        id: 'docker-installation',
        title: 'Docker Installation (Alternative)',
        level: 2,
      },
    ],
  },
  {
    id: 'workspace-setup',
    title: 'Workspace Setup',
    level: 1,
    children: [
      {
        id: 'colcon-build',
        title: 'Understanding colcon build',
        level: 2,
      },
      {
        id: 'source-setup',
        title: 'Sourcing Your Workspace',
        level: 2,
      },
    ],
  },
  {
    id: 'cli-tools',
    title: 'Essential CLI Tools',
    level: 1,
    children: [
      {
        id: 'ros2-node',
        title: 'ros2 node - Inspecting Nodes',
        level: 2,
      },
      {
        id: 'ros2-topic',
        title: 'ros2 topic - Working with Topics',
        level: 2,
      },
    ],
  },
];

// Example chapter metadata
const examplePreviousChapter: NavChapter | undefined = undefined; // First chapter

const exampleNextChapter: NavChapter = {
  title: 'Nodes, Topics, and Publishers/Subscribers',
  link: '/docs/module1/chapter2-topics',
};

/**
 * Complete example of Chapter 1 layout
 * This demonstrates all features of the ChapterPage component
 */
export default function ChapterLayoutExample(): JSX.Element {
  return (
    <ChapterPage
      moduleNumber={1}
      moduleName="ROS 2 Fundamentals"
      chapterNumber={1}
      chapterTitle="ROS 2 Installation and Workspace Setup"
      description="Learn to install ROS 2 Humble/Jazzy, create your first colcon workspace, and master essential ROS 2 CLI tools."
      bloomLevel="Understand"
      estimatedTime="1.5-2 hours"
      tableOfContents={exampleTOC}
      progress={0} // 0% - first chapter
      previousChapter={examplePreviousChapter}
      nextChapter={exampleNextChapter}
    >
      {/* ==================== CHAPTER CONTENT ==================== */}

      <h2 id="the-first-step">The First Step: Bringing ROS 2 to Life</h2>

      <p>
        Welcome to the foundational chapter of Module 1! Before we dive into the exciting world of robot communication and
        control, we need to set up our development environment. This chapter will guide you through installing ROS 2
        (Robot Operating System 2), creating your first workspace, and familiarizing yourself with the essential
        command-line interface (CLI) tools that are your window into a running ROS 2 system.
      </p>

      <p>
        A robust setup is crucial. While installation might seem like a mere formality, it's where many beginners
        encounter their first hurdles. We'll cover common operating systems and provide troubleshooting tips to ensure you
        have a smooth start.
      </p>

      <h3 id="learning-outcomes">Learning Outcomes</h3>

      <p>By the end of this chapter, you will be able to:</p>

      <ul>
        <li>
          <strong>Install</strong> ROS 2 Humble or Jazzy on Ubuntu/WSL2 without errors.
        </li>
        <li>
          <strong>Create</strong> a ROS 2 workspace using <code>colcon</code> and understand its directory structure.
        </li>
        <li>
          <strong>Use</strong> <code>ros2</code> CLI tools to inspect running nodes and topics.
        </li>
      </ul>

      <h2 id="ros2-fundamentals">ROS 2 Fundamentals</h2>

      <h3 id="what-is-ros">What is ROS 2?</h3>

      <p>
        ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools
        and libraries that help you build robot applications. The core idea is simple: robots are complex systems made up
        of many components (sensors, motors, vision algorithms, path planners, etc.) that need to communicate with each
        other.
      </p>

      <blockquote>
        <p>
          Think of ROS 2 as a "middleware" - software that sits between your application code and the hardware, managing
          all the complexity of inter-process communication, hardware drivers, and message passing.
        </p>
      </blockquote>

      <h3 id="architecture-overview">Architecture Overview</h3>

      <p>
        ROS 2 is built on a Distributed Data Service (DDS) standard, which provides a scalable, real-time, anonymous
        publish-subscribe pattern for data exchange.
      </p>

      <table>
        <thead>
          <tr>
            <th>Component</th>
            <th>Purpose</th>
            <th>Example</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>
              <strong>Node</strong>
            </td>
            <td>Executable process that does work</td>
            <td>Motor controller, camera driver, motion planner</td>
          </tr>
          <tr>
            <td>
              <strong>Topic</strong>
            </td>
            <td>Asynchronous one-way communication</td>
            <td>/camera/image, /motor/speed, /sensors/imu</td>
          </tr>
          <tr>
            <td>
              <strong>Service</strong>
            </td>
            <td>Synchronous request-response</td>
            <td>Gripper open/close, calibrate sensor</td>
          </tr>
          <tr>
            <td>
              <strong>Action</strong>
            </td>
            <td>Long-running, cancellable tasks</td>
            <td>Navigate to goal, pick and place</td>
          </tr>
        </tbody>
      </table>

      <h2 id="installation-guide">Installation Guide</h2>

      <h3 id="ubuntu-installation">Ubuntu 22.04 LTS Installation</h3>

      <p>
        Ubuntu 22.04 LTS (Jammy Jellyfish) is the recommended operating system for ROS 2 Humble. Follow these steps to
        install ROS 2 Humble:
      </p>

      <ol>
        <li>
          <strong>Set Up Your Locale</strong>: Ensure you have a UTF-8 locale.
          <pre>
            <code>{`sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8`}</code>
          </pre>
        </li>

        <li>
          <strong>Add ROS 2 Repository</strong>: Add the official ROS 2 apt repository.
          <pre>
            <code>{`sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`}</code>
          </pre>
        </li>

        <li>
          <strong>Install ROS 2</strong>: Install the full desktop distribution.
          <pre>
            <code>{`sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop python3-argcomplete`}</code>
          </pre>
        </li>
      </ol>

      <h3 id="wsl2-installation">Windows WSL2 Setup</h3>

      <p>
        If you're on Windows, use Windows Subsystem for Linux (WSL2) to run Ubuntu and install ROS 2 as described above.
      </p>

      <h3 id="docker-installation">Docker Installation (Alternative)</h3>

      <p>Alternatively, use Docker for a consistent ROS 2 environment:</p>

      <pre>
        <code>{`docker pull osrf/ros:humble-desktop
docker run -it osrf/ros:humble-desktop`}</code>
      </pre>

      <h2 id="workspace-setup">Workspace Setup</h2>

      <h3 id="colcon-build">Understanding colcon build</h3>

      <p>
        ROS 2 uses <strong>colcon</strong> (collective construction) as the build tool. A typical ROS 2 workspace has
        this structure:
      </p>

      <pre>
        <code>{`ros2_ws/
├── src/          # Your source code goes here
├── build/        # Build artifacts (auto-generated)
├── install/      # Installed packages (auto-generated)
└── log/          # Build logs (auto-generated)`}</code>
      </pre>

      <h3 id="source-setup">Sourcing Your Workspace</h3>

      <p>After building, always source your workspace before running ROS 2 commands:</p>

      <pre>
        <code>{`source ~/ros2_ws/install/setup.bash`}</code>
      </pre>

      <p>To make this permanent, add it to your ~/.bashrc:</p>

      <pre>
        <code>{`echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc`}</code>
      </pre>

      <h2 id="cli-tools">Essential CLI Tools</h2>

      <h3 id="ros2-node">ros2 node - Inspecting Nodes</h3>

      <p>List all running nodes:</p>

      <pre>
        <code>{`ros2 node list`}</code>
      </pre>

      <p>Get info about a specific node:</p>

      <pre>
        <code>{`ros2 node info /node_name`}</code>
      </pre>

      <h3 id="ros2-topic">ros2 topic - Working with Topics</h3>

      <p>List all active topics:</p>

      <pre>
        <code>{`ros2 topic list`}</code>
      </pre>

      <p>Echo messages from a topic in real-time:</p>

      <pre>
        <code>{`ros2 topic echo /topic_name`}</code>
      </pre>

      <h2>Hands-On Exercise: Your First ROS 2 Installation</h2>

      <h3>Objective</h3>
      <p>Install ROS 2 Humble and verify it works with the built-in talker/listener demo.</p>

      <h3>Steps</h3>
      <ol>
        <li>Complete the installation for your OS above</li>
        <li>
          Open two terminals and run:
          <pre>
            <code>{`# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_cpp listener`}</code>
          </pre>
        </li>
        <li>You should see messages flowing from talker to listener</li>
        <li>Celebrate! You have a working ROS 2 installation</li>
      </ol>

      <h3>Success Criteria</h3>
      <ul>
        <li>ROS 2 installs without errors</li>
        <li>Talker and listener communicate successfully</li>
        <li>You understand the difference between topics and services</li>
      </ul>

      <h2>Troubleshooting</h2>

      <h3>Issue: "command not found: ros2"</h3>
      <p>
        <strong>Solution:</strong> You haven't sourced your workspace. Run:
      </p>
      <pre>
        <code>{`source /opt/ros/humble/setup.bash`}</code>
      </pre>

      <h3>Issue: Permission denied when installing</h3>
      <p>
        <strong>Solution:</strong> Make sure you're using <code>sudo</code> for system-level apt commands.
      </p>

      <h2>Summary</h2>

      <p>
        Congratulations! You've installed ROS 2 and verified it works. You now understand the basic concepts of nodes,
        topics, and the ROS 2 architecture. In the next chapter, you'll dive deep into the publish-subscribe pattern and
        write your own ROS 2 nodes.
      </p>
    </ChapterPage>
  );
}
