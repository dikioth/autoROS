import { useAnimationFrame } from "@cruise-automation/hooks";
import joystick from './Joystick.svg'; // Tell webpack this JS file uses this image
import dest_icon from './Destination.svg'; // Tell webpack this JS file uses this image
import destination_icon from './Destination.svg'; // Tell webpack this JS file uses this image

import React, { useState } from "react";
import ROSLIB from "roslib";
import Worldview, { Arrows, Text, Lines, Cubes, Grid, Axes, Points } from "regl-worldview";
import { Button, Navbar, Nav, ButtonGroup, Form, OverlayTrigger, Popover, ButtonToolbar } from 'react-bootstrap';

let anchorsArr = []
let pointslength = 0;
let tagpos = { x: 5, y: 5 };
let tagOrientation = { x: 0, y: 0, z: 0, w: 0 }
var teleopPub;

function App() {
  const [count, setCount] = useState(0);
  const [commandMsgs, setCommandMsgs] = useState([]);

  const validatedCount = isNaN(count) || count < 1 ? 6 : Math.round(count);
  useAnimationFrame(
    () => {
      // update count before each browser repaint
      const newCount = count + 1;
      setCount(newCount);
    },
    false,
    []
  );

  const poseArrow1 = {
    pose: {
      orientation: { x: tagOrientation.x, y: tagOrientation.y, z: tagOrientation.z, w: tagOrientation.w },
      position: { x: tagpos.x, y: tagpos.y, z: 0 },
    },
    scale: { x: 10, y: 0.5, z: 0.5 },
    color: { r: 1, g: 0, b: 1, a: 1 },
  };




  const points = [];
  const step = 0.5;
  for (let i = 0; i < 100; i++) {
    for (let j = 0; j < 100; j++) {
      points.push({ x: i * step, y: j * step, z: 0 });

    }
  }
  pointslength = points.length;

  const pointsMarker = {
    // Unique identifier for the object that contains multiple instances.
    id: 1000,
    pose: {
      orientation: { x: 0, y: 0, z: 0, w: 1 },
      position: { x: 0, y: 0, z: -0.5 },
    },
    scale: { x: 10, y: 10, z: 1 },
    color: { r: 0, g: 0, b: 0, a: 0 },
    points,
    info: "an instanced point",
  };

  const keyboard_popover = (
    <Popover id="popover-basic">
      <Popover.Title as="h3">Popover right</Popover.Title>
      <Popover.Content>
        And here's some <strong>amazing</strong> content. It's very engaging.
        right?
      </Popover.Content>
    </Popover>
  );

  const KeyboardPop = () => (
    <OverlayTrigger trigger="click" placement="bottom" overlay={keyboard_popover}>
      <Button variant="success">Click me to see</Button>
    </OverlayTrigger>
  );

  function handleKeyCtrl(ev) {
    let instruction = ev.target.getAttribute('id');
    // publish control instruction to ROS.
    switch (instruction) {
      case "forward":
        teleopPub.publish(new ROSLIB.Message({ data: "i" }));
        break;
      case "backward":
        teleopPub.publish(new ROSLIB.Message({ data: "," }));
        break;
      case "left":
        teleopPub.publish(new ROSLIB.Message({ data: "j" }));
        break;
      case "right":
        teleopPub.publish(new ROSLIB.Message({ data: "l" }));
        break;
      case "stop":
        teleopPub.publish(new ROSLIB.Message({ data: "k" }));
        break;

      default:
        break;
    }

  }

  // Return
  return (
    <div>
      <Navbar bg="primary" variant="dark" sticky="top">
        <Navbar.Toggle aria-controls="responsive-navbar-nav" />
        <Navbar.Brand href="#home">AUTOROS</Navbar.Brand>

        <Nav className="mr-auto">
          <Nav.Link href="#home">Live</Nav.Link>
          <Nav.Link href="#features">Dashboard</Nav.Link>
        </Nav>
        <Button variant="outline-primary"><img src={dest_icon} width="30px" /></Button>
        <ButtonToolbar>
          <OverlayTrigger
            trigger="click"
            key={"bottom"}
            placement={"bottom"}
            overlay={
              <Popover id={`popover-positioned-${"bottom"}`}>
                <Popover.Title as="h3">{`Press the keys`}</Popover.Title>
                <Popover.Content>
                  <ButtonGroup vertical aria-label="Basic example">
                    <Button id="forward" variant="secondary" onClick={handleKeyCtrl}>Forward</Button>
                    <ButtonGroup aria-label="Basic example">
                      <Button id="left" variant="secondary" onClick={handleKeyCtrl}>Left</Button>
                      <Button id="stop" variant="secondary" onClick={handleKeyCtrl}>stop</Button>
                      <Button id="right" variant="secondary" onClick={handleKeyCtrl} >Right</Button>
                    </ButtonGroup>
                    <Button id="backward" variant="secondary" onClick={handleKeyCtrl}>Backward</Button>
                  </ButtonGroup>
                </Popover.Content>
              </Popover>
            }
          >
            <Button variant="outline-primary"><img src={joystick} width="30px" /></Button>
          </OverlayTrigger>
        </ButtonToolbar>

      </Navbar>

      <Worldview onClick={onWorldviewClick} enableStackedObjectEvents defaultCameraState={{
        distance: 40,
        phi: 0.5,
        target: [tagpos.x, tagpos.y, 0],
      }} style={{ width: "100vw", height: "90vh" }} >



        <Grid count={100} />



        <Axes />
        <Cubes>
          {[
            {
              pose: {
                orientation: tagOrientation,
                // position the cube at the center
                position: { x: tagpos.x, y: tagpos.y, z: 0 },
              },
              scale: { x: 1, y: 1, z: 1 },
              // rgba values are between 0 and 1 (inclusive)
              color: { r: 1, g: 0, b: 0, a: 1 },
            }
          ]}
        </Cubes>

        {
          anchorsArr.map((anchor, id) => (

            <Cubes key={id}>
              {[
                {
                  pose: {
                    orientation: { x: 0, y: 0, z: 0, w: 1 },
                    position: { x: 10 * anchor.x, y: 10 * anchor.y, z: 0 },
                  },
                  scale: { x: 1, y: 1, z: 1 },
                  color: { r: 1, g: 0, b: 0, a: 1 },
                },
              ]}
            </Cubes>
          ))
        }

        {
          anchorsArr.map((anchor, id) => (
            <Text key={id}>{[{
              name: "",
              text: anchor.header.frame_id,
              color: { r: 1, g: 1, b: 1, a: 1 },
              pose: {
                orientation: { x: 0, y: 0, z: 0, w: 1 },
                position: { x: 10 * anchor.x, y: 10 * anchor.y, z: 0 },
              },
              scale: { x: 1, y: 1, z: 1 },
              // uncomment colors and remove autoBackgroundColor prop to set text and background colors
              // colors: [{ r: 1, g: 1, b: 1, a: 1 }, { r: 1, g: 0, b: 0, a: 0.8 }],
            }]}</Text>
          ))
        }
        <Arrows>{[poseArrow1]}</Arrows>

        {
          anchorsArr.map((anchor, id) => (
            <Lines key={id}>{[{
              pose: {
                position: { x: 0, y: 0, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 0 }
              },
              scale: { x: 0.1, y: 0.1, z: 0.1 },
              color: { r: 0, g: 1, b: 0, a: 1 },
              points: [
                { x: 10 * anchor.x, y: 10 * anchor.y, z: 0 },
                { x: tagpos.x, y: tagpos.y, z: 0 }
              ],
              colors: { r: 1, g: 1, b: 1, a: 1 }
            }]}</Lines>
          ))
        }

        <Points>{[pointsMarker]}</Points>

      </Worldview >

    </div >
  );


  function onWorldviewClick(ev, { objects }) {
    const messages = objects.map(({ object, instanceIndex }) => {
      if (object.points && instanceIndex >= 0 && instanceIndex <= points.length) {
        return `Clicked ${object.info}. The objectId is ${object.id} and its position is ${JSON.stringify(
          object.points[instanceIndex]
        )}`;
      }
      return `Clicked ${object.info}. The objectId is ${object.id} and its position is ${JSON.stringify(
        object.pose.position
      )}`;
    });
    setCommandMsgs(messages);
  }
}




function ConnectROSbridge() {
  // Connecting to ROS websockets
  var ros = new ROSLIB.Ros({
    url: "ws://localhost:9090"
  });

  ros.on("connection", function () {
    console.log("Connected to websocket server.");

    // create a listener
    var AnchorsListener = new ROSLIB.Topic({
      ros: ros,
      name: "/dwm1001/anchors",
      messageType: "localizer_dwm1001/AnchorArray"
    });


    var tagListener = new ROSLIB.Topic({
      ros: ros,
      name: "/dwm1001/tag_odom",
      messageType: "nav_msgs/Odometry"
    });

    teleopPub = new ROSLIB.Topic({
      ros: ros,
      name: '/autoros/teleop_string',
      messageType: 'std_msgs/String'
    });

    teleopPub.publish(new ROSLIB.Message({ data: "k" }));

    AnchorsListener.subscribe(function (message) {
      anchorsArr = []
      message.anchors.forEach(function (anchor) {
        anchorsArr.push(anchor)
      });
      // listener.unsubscribe();
    });
    // subscribe to Tag topic
    tagListener.subscribe(function (message) {
      tagpos.x = 10 * message.pose.pose.position.x;
      tagpos.y = 10 * message.pose.pose.position.y;

      tagOrientation.x = message.pose.pose.orientation.x;
      tagOrientation.y = message.pose.pose.orientation.y;
      tagOrientation.z = message.pose.pose.orientation.z;
      tagOrientation.w = message.pose.pose.orientation.w;

      // listener.unsubscribe();
    });
  });

  ros.on("error", function (error) {
    console.log("Error connecting to websocket server: ", error);

  });

  ros.on("close", function () {
    console.log("Connection to websocket server closed.");
  });
}
ConnectROSbridge();
export default App;