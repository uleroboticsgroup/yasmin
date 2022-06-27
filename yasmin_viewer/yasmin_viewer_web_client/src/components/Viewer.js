import React from "react";
import TextField from "@mui/material/TextField";
import Autocomplete from "@mui/material/Autocomplete";
import Grid from "@mui/material/Grid";
import FSM from "./FSM";

class Viewer extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      fsm_list: [],
      fsm_name_list: [],
      current_fsm_data: undefined,
      current_fsm: "ALL",
    };
  }

  get_fsms() {
    fetch("/get_fsms")
      .then((res) => res.json())
      .then((data) => {
        let fsm_list = [];
        let fsm_name_list = ["ALL"];

        for (let key in data) {
          fsm_name_list.push(key);
          fsm_list.push(data[key]);
        }

        this.setState({
          fsm_list: fsm_list,
          fsm_name_list: fsm_name_list,
        });
      });
  }

  get_fsm() {
    fetch("/get_fsm/" + this.state.current_fsm)
      .then((res) => res.json())
      .then((data) => {
        if (Object.keys(data).length !== 0) {
          this.setState({ current_fsm_data: data });
        } else {
          this.setState({ current_fsm: "ALL" });
        }
      });
  }

  componentDidMount() {
    this.interval = setInterval(() => {
      if (this.state.current_fsm === "ALL") {
        this.get_fsms();
      } else {
        this.get_fsm();
      }
    }, 250);
  }

  componentWillUnmount() {
    clearInterval(this.interval);
  }

  render() {
    return (
      <div
        style={{
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
        }}
      >
        <Grid container spacing={3}>
          <Grid item xs={12}>
            <div
              style={{
                display: "flex",
                justifyContent: "center",
                alignItems: "center",
              }}
            >
              <Autocomplete
                id="combo-box"
                disableClearable={true}
                options={this.state.fsm_name_list}
                getOptionLabel={(option) => option}
                isOptionEqualToValue={(option, value) => option === value}
                defaultValue={"ALL"}
                value={this.state.current_fsm}
                onChange={(event, value) => {
                  this.setState({ current_fsm: value });
                }}
                style={{ width: 300 }}
                renderInput={(params) => (
                  <TextField {...params} label="FSM" variant="outlined" />
                )}
              />
            </div>
          </Grid>

          {this.state.current_fsm === "ALL" ? (
            this.state.fsm_list.map((fsm) => {
              return (
                <Grid item xs={6} key={fsm[0].name}>
                  <FSM fsm_data={fsm} alone={false} />
                </Grid>
              );
            })
          ) : (
            <Grid item xs={12}>
              <FSM fsm_data={this.state.current_fsm_data} alone={true} />
            </Grid>
          )}
        </Grid>
      </div>
    );
  }
}

export default Viewer;
