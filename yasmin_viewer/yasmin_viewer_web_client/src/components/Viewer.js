import React from 'react';
import TextField from '@material-ui/core/TextField';
import Autocomplete from '@material-ui/lab/Autocomplete';
import FSM from "./FSM"
import Grid from '@material-ui/core/Grid';

class Viewer extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            fsm_list: [],
            current_fsm: { fsm_name: "ALL" }
        };
    }

    get_fsms() {
        fetch('/get_fsms').then(res => res.json()).then(data => {
            var aux_fsm_list = [{ fsm_name: "ALL" }]

            for (var key in data) {
                var value = data[key];
                aux_fsm_list.push(value);
            }

            this.setState({ fsm_list: aux_fsm_list })


        });
    }

    get_fsm() {
        fetch('/get_fsm/' + this.state.current_fsm.fsm_name).then(res => res.json()).then(data => {
            this.setState({ current_fsm: data })
        });
    }

    componentDidMount() {
        this.interval = setInterval(() => {
            if (this.state.current_fsm.fsm_name === "ALL") {
                this.get_fsms()
            } else {
                this.get_fsm()
            }
        }, 250);
    }

    componentWillUnmount() {
        clearInterval(this.interval);
    }


    render() {




        return (< div style={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
        }}>


            <Grid container spacing={3}>
                <Grid item xs={12}>
                    <div style={{
                        display: "flex",
                        justifyContent: "center",
                        alignItems: "center",
                    }}>
                        <Autocomplete
                            id="combo-box"
                            disableClearable="true"
                            options={this.state.fsm_list}
                            getOptionLabel={(option) => option.fsm_name}
                            getOptionSelected={(option, value) => option.fsm_name === value.fsm_name}
                            defaultValue={{ fsm_name: "ALL" }}
                            onChange={(event, value) => { this.setState({ current_fsm: value }); }}
                            style={{ width: 300 }}
                            renderInput={(params) => <TextField {...params} label="FSM" variant="outlined" />}
                        />
                    </div>
                </Grid>

                {this.state.current_fsm.fsm_name === "ALL" ? (

                    this.state.fsm_list.map((fsm) => {
                        if (fsm.fsm_name !== "ALL") {
                            return (
                                < Grid item xs={6} key={fsm.fsm_name} >
                                    < FSM fsm_data={fsm} />
                                </Grid>)
                        }
                    }
                    )

                ) : (
                        < Grid item xs={12} >
                            < FSM fsm_data={this.state.current_fsm} />
                        </Grid>
                    )}
            </Grid>
        </div >)
    }
}

export default Viewer;
