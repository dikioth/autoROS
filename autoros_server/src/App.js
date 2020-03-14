import { useAnimationFrame } from "@cruise-automation/hooks";
import joystick from "./Joystick.svg"; // Tell webpack this JS file uses this image
import dest_icon from "./Destination.svg"; // Tell webpack this JS file uses this image
import Dashboard from "./components/Dashboard";

import React, { useState } from "react";
import ROSLIB from "roslib";
import Worldview, {
  Arrows,
  Text,
  Lines,
  Cubes,
  Grid,
  Axes,
  PolygonBuilder,
  DrawPolygons
} from "regl-worldview";
import {
  Button,
  Navbar,
  Nav,
  ButtonGroup,
  Tab,
  Tabs,
  OverlayTrigger,
  Popover,
  ButtonToolbar,
  Modal
} from "react-bootstrap";

let anchorsArr = [];
let pointslength = 0;
let tagpos = { x: 5, y: 5 };
let tagOrientation = { x: 0, y: 0, z: 0, w: 0 };
var teleopPub;
let rosConnectionStablished = false;

let targetPositionPolygon = new PolygonBuilder();
function App() {
  // TODO: Use OOP.

  const [count, setCount] = useState(0);
  const [commandMsgs, setCommandMsgs] = useState([]);
  const [drawOnCanvas, setDrawOnCanvas] = useState(false);

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
      orientation: {
        x: tagOrientation.x,
        y: tagOrientation.y,
        z: tagOrientation.z,
        w: tagOrientation.w
      },
      position: { x: tagpos.x, y: tagpos.y, z: 0 }
    },
    scale: { x: 10, y: 0.5, z: 0.5 },
    color: { r: 1, g: 0, b: 1, a: 1 }
  };

  const points = [];
  const step = 0.5;
  for (let i = 0; i < 100; i++) {
    for (let j = 0; j < 100; j++) {
      points.push({ x: i * step, y: j * step, z: 0 });
    }
  }
  pointslength = points.length;

  const keyboard_popover = (
    <Popover id="popover-basic">
      <Popover.Title as="h3">Popover right</Popover.Title>
      <Popover.Content>
        And here's some <strong>amazing</strong> content. It's very engaging.
        right?
      </Popover.Content>
    </Popover>
  );

  function handleKeyCtrl(ev) {
    let instruction = ev.target.getAttribute("id");
    // publish control instruction to ROS.

    if (rosConnectionStablished) {
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
  }

  function _handleEvent(eventName, ev, args) {
    targetPositionPolygon[eventName](ev, args);
    console.log(targetPositionPolygon.polygons[0].points);
  }

  function onDoubleClick(ev, args) {
    _handleEvent("onDoubleClick", ev, args);
  }
  function onMouseDown(ev, args) {
    _handleEvent("onMouseDown", ev, args);
  }
  function onMouseMove(ev, args) {
    _handleEvent("onMouseMove", ev, args);
  }
  function onMouseUp(ev, args) {
    _handleEvent("onMouseUp", ev, args);
  }

  function websocketModal(props) {
    return (
      <Modal
        {...props}
        size="lg"
        aria-labelledby="contained-modal-title-vcenter"
        centered
      >
        <Modal.Header closeButton>
          <Modal.Title id="contained-modal-title-vcenter">
            Modal heading
          </Modal.Title>
        </Modal.Header>
        <Modal.Body>
          <h4>Centered Modal</h4>
          <p>
            Cras mattis consectetur purus sit amet fermentum. Cras justo odio,
            dapibus ac facilisis in, egestas eget quam. Morbi leo risus, porta
            ac consectetur ac, vestibulum at eros.
          </p>
        </Modal.Body>
        <Modal.Footer>
          <Button onClick={props.onHide}>Close</Button>
        </Modal.Footer>
      </Modal>
    );
  }

  // Return
  return (
    <div>
      <Navbar bg="primary" variant="dark" sticky="top">
        <Navbar.Toggle aria-controls="responsive-navbar-nav" />
        <Navbar.Brand href="#home">AUTOROS</Navbar.Brand>

        <Nav className="mr-auto">
          <Nav.Link href="#home">Github</Nav.Link>
        </Nav>

        <ButtonToolbar>
          <OverlayTrigger
            trigger="click"
            key={"bottom"}
            placement={"bottom"}
            overlay={
              <Popover id={`popover-positioned-${"bottom"}`}>
                <Popover.Title as="h3">{`Draw the path and click Run`}</Popover.Title>
                <Popover.Content>
                  <ButtonGroup aria-label="Basic example">
                    <Button id="btn_reset" variant="secondary">
                      Reset
                    </Button>
                    {"  "}
                    <Button id="btn_run" variant="primary">
                      Run
                    </Button>
                  </ButtonGroup>
                </Popover.Content>
              </Popover>
            }
          >
            <Button variant="light">
              <img
                src={dest_icon}
                width="25px"
                onClick={() => setDrawOnCanvas(!drawOnCanvas)}
              />
            </Button>
          </OverlayTrigger>
        </ButtonToolbar>
        {"."}
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
                    <Button
                      id="forward"
                      variant="secondary"
                      onClick={handleKeyCtrl}
                    >
                      Forward
                    </Button>
                    <ButtonGroup aria-label="Basic example">
                      <Button
                        id="left"
                        variant="secondary"
                        onClick={handleKeyCtrl}
                      >
                        Left
                      </Button>
                      <Button
                        id="stop"
                        variant="secondary"
                        onClick={handleKeyCtrl}
                      >
                        stop
                      </Button>
                      <Button
                        id="right"
                        variant="secondary"
                        onClick={handleKeyCtrl}
                      >
                        Right
                      </Button>
                    </ButtonGroup>
                    <Button
                      id="backward"
                      variant="secondary"
                      onClick={handleKeyCtrl}
                    >
                      Backward
                    </Button>
                  </ButtonGroup>
                </Popover.Content>
              </Popover>
            }
          >
            <Button variant="light">
              <img src={joystick} width="25px" />
            </Button>
          </OverlayTrigger>
        </ButtonToolbar>
      </Navbar>

      <Tabs bg="primary" id="controlled-tab-example">
        <Tab eventKey="home" title="Live">
          <Worldview
            onDoubleClick={onDoubleClick}
            onMouseDown={onMouseDown}
            onMouseMove={onMouseMove}
            onMouseUp={onMouseUp}
            onClick={onWorldviewClick}
            enableStackedObjectEvents
            defaultCameraState={{
              distance: 40,
              phi: 0.5,
              target: [tagpos.x, tagpos.y, 0]
            }}
            style={{ width: "100vw", height: "85vh" }}
          >
            <Grid count={100} />

            <Axes />
            <Cubes>
              {[
                {
                  pose: {
                    orientation: tagOrientation,
                    // position the cube at the center
                    position: { x: tagpos.x, y: tagpos.y, z: 0 }
                  },
                  scale: { x: 1, y: 1, z: 1 },
                  // rgba values are between 0 and 1 (inclusive)
                  color: { r: 1, g: 0, b: 0, a: 1 }
                }
              ]}
            </Cubes>

            {anchorsArr.map((anchor, id) => (
              <Cubes key={id}>
                {[
                  {
                    pose: {
                      orientation: { x: 0, y: 0, z: 0, w: 1 },
                      position: { x: 10 * anchor.x, y: 10 * anchor.y, z: 0 }
                    },
                    scale: { x: 1, y: 1, z: 1 },
                    color: { r: 1, g: 0, b: 0, a: 1 }
                  }
                ]}
              </Cubes>
            ))}

            {anchorsArr.map((anchor, id) => (
              <Text key={id}>
                {[
                  {
                    name: "",
                    text: anchor.header.frame_id,
                    color: { r: 1, g: 1, b: 1, a: 1 },
                    pose: {
                      orientation: { x: 0, y: 0, z: 0, w: 1 },
                      position: { x: 10 * anchor.x, y: 10 * anchor.y, z: 0 }
                    },
                    scale: { x: 1, y: 1, z: 1 }
                    // uncomment colors and remove autoBackgroundColor prop to set text and background colors
                    // colors: [{ r: 1, g: 1, b: 1, a: 1 }, { r: 1, g: 0, b: 0, a: 0.8 }],
                  }
                ]}
              </Text>
            ))}
            <Arrows>{[poseArrow1]}</Arrows>

            {anchorsArr.map((anchor, id) => (
              <Lines key={id}>
                {[
                  {
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
                  }
                ]}
              </Lines>
            ))}

            <DrawPolygons>{targetPositionPolygon.polygons}</DrawPolygons>

            {/* <Points>{[pointsMarker]}</Points> */}
          </Worldview>
        </Tab>
        <Tab eventKey="profile" title="DASHBOARD">
          <Dashboard />)
        </Tab>
      </Tabs>
    </div>
  );

  function onWorldviewClick(ev, { objects }) {
    const messages = objects.map(({ object, instanceIndex }) => {
      if (
        object.points &&
        instanceIndex >= 0 &&
        instanceIndex <= points.length
      ) {
        // return object.points[instanceIndex]
        // return `Clicked ${object.info}. The objectId is ${object.id} and its position is ${JSON.stringify(
        //   object.points[instanceIndex]
        // )}`;
      }
      // return object.pose.position;
      // return `Clicked ${object.info}. The objectId is ${object.id} and its position is ${JSON.stringify(
      //   object.pose.position
      // )}`;
      console.log(object.points[instanceIndex]);
    });
    // setCommandMsgs(messages);
  }
}

function ConnectROSbridge() {
  // Connecting to ROS websockets
  var ros = new ROSLIB.Ros({
    url: "ws://localhost:9090"
  });

  ros.on("connection", function() {
    console.log("Connected to websocket server.");
    rosConnectionStablished = true;

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
      name: "/autoros/teleop_string",
      messageType: "std_msgs/String"
    });

    teleopPub.publish(new ROSLIB.Message({ data: "k" }));

    AnchorsListener.subscribe(function(message) {
      anchorsArr = [];
      message.anchors.forEach(function(anchor) {
        anchorsArr.push(anchor);
      });
      // listener.unsubscribe();
    });
    // subscribe to Tag topic
    tagListener.subscribe(function(message) {
      tagpos.x = 10 * message.pose.pose.position.x;
      tagpos.y = 10 * message.pose.pose.position.y;

      tagOrientation.x = message.pose.pose.orientation.x;
      tagOrientation.y = message.pose.pose.orientation.y;
      tagOrientation.z = message.pose.pose.orientation.z;
      tagOrientation.w = message.pose.pose.orientation.w;

      // listener.unsubscribe();
    });
  });

  ros.on("error", function(error) {
    console.log("Error connecting to websocket server: ", error);
  });

  ros.on("close", function() {
    console.log("Connection to websocket server closed.");
    rosConnectionStablished = false;
  });
}
ConnectROSbridge();
export default App;
