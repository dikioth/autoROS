import React, { Component } from "react";
import { Line } from "react-chartjs-2";
import { Container, Row, Col } from "react-bootstrap";

class Dashboard extends Component {
  constructor(props) {
    super(props);
    this.state = {
      chartdata: {
        labels: [
          ["June", "2015"],
          "July",
          "August",
          "September",
          "October",
          "November",
          "December",
          ["January", "2016"],
          "February",
          "March",
          "April",
          "May"
        ],
        datasets: [
          {
            label: "x position",
            fill: false,
            backgroundColor: "#ff2e0a",
            borderColor: "#ff2e0a",
            data: [
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor()
            ]
          },
          {
            label: "x estimation",
            fill: false,
            backgroundColor: "#343e9a",
            borderColor: "#343e9a",
            data: [
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor(),
              this.randomScalingFactor()
            ]
          }
        ]
      },
      chartoptions: {
        responsive: true,
        title: {
          display: true,
          text: "Chart with Multiline Labels"
        }
      }
    };
  }

  randomScalingFactor() {
    return Math.round(Math.random() * 100);
  }

  render() {
    return (
      <Container>
        <Row>
          <Col>
            <Line data={this.state.chartdata} options={this.state.options} />
          </Col>
          <Col>
            {" "}
            <Line data={this.state.chartdata} options={this.state.options} />
          </Col>
        </Row>
        <Row>
          <Col>
            <Line data={this.state.chartdata} options={this.state.options} />
          </Col>
          <Col>
            {" "}
            <Line data={this.state.chartdata} options={this.state.options} />
          </Col>
        </Row>
      </Container>
    );
  }
}

export default Dashboard;
