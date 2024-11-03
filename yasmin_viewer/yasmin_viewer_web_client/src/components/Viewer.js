// Copyright (C) 2023  Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

import React from "react";
import Grid from "@mui/material/Grid";
import FSM from "./FSM";
import TopAppBar from "./TopAppBar";

class Viewer extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      fsm_list: [],
      fsm_name_list: [],
      current_fsm: "ALL",
      hide_nested_fsm: false,
      show_only_active_fsms: false,
    };

    this.handle_current_fsm = this.handle_current_fsm.bind(this);
    this.handle_hide_nested_fsm = this.handle_hide_nested_fsm.bind(this);
    this.handle_show_only_active_fsms =
      this.handle_show_only_active_fsms.bind(this);
  }

  get_fsms() {
    fetch("/get_fsms")
      .then((res) => res.json())
      .then((data) => {
        let fsm_list = [];
        let fsm_name_list = ["ALL"];
        let current_fsm = this.state.current_fsm;

        for (let key in data) {
          fsm_name_list.push(key);
          fsm_list.push(data[key]);
        }

        if (!fsm_name_list.includes(current_fsm)) {
          current_fsm = "ALL";
        }

        // Update state only if there's an actual difference
        if (JSON.stringify(fsm_list) !== JSON.stringify(this.state.fsm_list)) {
          this.setState({
            fsm_list: fsm_list,
            fsm_name_list: fsm_name_list,
            current_fsm: current_fsm,
          });
        }
      });
  }

  componentDidMount() {
    this.interval = setInterval(() => {
      this.get_fsms();
    }, 250);
  }

  componentWillUnmount() {
    clearInterval(this.interval);
  }

  handle_current_fsm(current_fsm) {
    if (this.state.current_fsm !== current_fsm) {
      this.setState({ current_fsm: current_fsm });
    }
  }

  handle_hide_nested_fsm(hide_nested_fsm) {
    if (this.state.hide_nested_fsm !== hide_nested_fsm) {
      this.setState({ hide_nested_fsm: hide_nested_fsm });
    }
  }

  handle_show_only_active_fsms(show_only_active_fsms) {
    if (this.state.show_only_active_fsms !== show_only_active_fsms) {
      this.setState({ show_only_active_fsms: show_only_active_fsms });
    }
  }

  shouldComponentUpdate(nextProps, nextState) {
    return (
      JSON.stringify(this.state) !== JSON.stringify(nextState) ||
      JSON.stringify(this.props) !== JSON.stringify(nextProps)
    );
  }

  render() {
    return (
      <div>
        <TopAppBar
          fsm_name_list={this.state.fsm_name_list}
          current_fsm={this.state.current_fsm}
          handle_current_fsm={this.handle_current_fsm}
          handle_hide_nested_fsm={this.handle_hide_nested_fsm}
          handle_show_only_active_fsms={this.handle_show_only_active_fsms}
        />

        <div
          style={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            minHeight: "700px",
          }}
        >
          <Grid container spacing={3}>
            {this.state.fsm_list.map((fsm) => {
              if (
                (this.state.show_only_active_fsms &&
                  fsm[0].current_state !== -1) ||
                (!this.state.show_only_active_fsms &&
                  this.state.current_fsm === fsm[0].name) ||
                this.state.current_fsm === "ALL"
              ) {
                return (
                  <Grid
                    item
                    xs={this.state.current_fsm === "ALL" ? 6 : 12}
                    key={
                      fsm[0].name +
                      this.state.current_fsm +
                      this.state.hide_nested_fsm
                    }
                  >
                    <FSM
                      fsm_data={fsm}
                      alone={false}
                      hide_nested_fsm={this.state.hide_nested_fsm}
                    />
                  </Grid>
                );
              }
              return null;
            })}
          </Grid>
        </div>
      </div>
    );
  }
}

export default Viewer;
