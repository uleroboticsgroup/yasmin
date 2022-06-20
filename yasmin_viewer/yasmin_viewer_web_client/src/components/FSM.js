import React from "react";

import CytoscapeComponent from "react-cytoscapejs";
//import dagre from "cytoscape-dagre";
import klay from "cytoscape-klay";
import cytoscape from "cytoscape";
//import cola from "cytoscape-cola";

import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";
import Grid from "@mui/material/Grid";

class FSM extends React.Component {
  prepare_graph(fsm_data) {
    let nodes = [];
    let edges = [];

    let fsms_stack = [];
    let fsm_outcomes_stack = [];
    let state_amount_stack = [];
    let current_state_path = fsm_data.fsm_name + fsm_data.current_state;

    let fsm_structure = fsm_data.fsm_structure;
    let states = Array.from(fsm_structure.states);

    fsms_stack.push(fsm_data.fsm_name);
    state_amount_stack.push(states.length);

    let aux_outcomes = [];
    let outcomes = fsm_structure.final_outcomes;

    for (let key in outcomes) {
      let outcome = outcomes[key];

      aux_outcomes.push(outcome);

      nodes.push({
        data: {
          id: fsms_stack[fsms_stack.length - 1] + outcome,
          label: outcome,
          type: "outcome",
          width: outcome.length * 12,
          height: (outcome.length * 12) / 2,
        },
      });
    }

    fsm_outcomes_stack.push(aux_outcomes);

    while (states.length > 0) {
      let state = states.pop();
      let parent = "";
      let type = "node";
      let height = 50;
      let width = 100;

      //// NODE ////
      if (fsm_data.fsm_name !== fsms_stack[fsms_stack.length - 1]) {
        parent = fsms_stack[fsms_stack.length - 1];
      }

      if (
        current_state_path ===
        fsms_stack[fsms_stack.length - 1] + state.state_name
      ) {
        if (!state.is_fsm) {
          type = "current_state";
        } else {
          current_state_path = current_state_path + state.current_state;
          type = "current_fsm";
        }
      } else {
        if (state.is_fsm) {
          type = "fsm";
        }
      }

      if (height < state.state_name.length * 6) {
        height = state.state_name.length * 6;
      }

      if (width < state.state_name.length * 12) {
        width = state.state_name.length * 12;
      }

      nodes.push({
        data: {
          id: fsms_stack[fsms_stack.length - 1] + state.state_name,
          parent: parent,
          label: state.state_name,
          type: type,
          width: width,
          height: height,
        },
      });

      state_amount_stack[state_amount_stack.length - 1] -= 1;

      //// TRANSITIONS ////
      for (let key in state.outcomes) {
        let outcome = state.outcomes[key];
        let source = fsms_stack[fsms_stack.length - 1] + state.state_name;

        if (state.is_fsm) {
          source =
            fsms_stack[fsms_stack.length - 1] + state.state_name + outcome;
        }

        if (state.transitions.hasOwnProperty(outcome)) {
          edges.push({
            data: {
              id:
                fsms_stack[fsms_stack.length - 1] +
                state.state_name +
                outcome +
                state.transitions[outcome],
              source: source,
              target:
                fsms_stack[fsms_stack.length - 1] + state.transitions[outcome],
              label: outcome,
            },
          });
        } else {
          if (
            fsm_outcomes_stack[fsm_outcomes_stack.length - 1].includes(outcome)
          ) {
            edges.push({
              data: {
                id:
                  fsms_stack[fsms_stack.length - 1] +
                  state.state_name +
                  outcome +
                  outcome,
                source: source,
                target: fsms_stack[fsms_stack.length - 1] + outcome,
                label: outcome,
              },
            });
          }
        }
      }

      //// STATE IS FSM ////
      if (state.is_fsm) {
        aux_outcomes = [];
        for (let key in state.outcomes) {
          let outcome = state.outcomes[key];
          aux_outcomes.push(outcome);

          if (state.is_fsm) {
            nodes.push({
              data: {
                id:
                  fsms_stack[fsms_stack.length - 1] +
                  state.state_name +
                  outcome,
                label: outcome,
                type: "outcome",
                parent: fsms_stack[fsms_stack.length - 1] + state.state_name,
                width: outcome.length * 12,
                height: (outcome.length * 12) / 2,
              },
            });
          }
        }

        fsms_stack.push(fsms_stack[fsms_stack.length - 1] + state.state_name);
        fsm_outcomes_stack.push(aux_outcomes);
        state_amount_stack.push(state.states.length);

        for (let key in state.states) {
          states.push(state.states[key]);
        }
      }

      //// CLEANING STACKS ////
      while (state_amount_stack[state_amount_stack.length - 1] === 0) {
        state_amount_stack.pop();
        fsms_stack.pop();
        fsm_outcomes_stack.pop();
      }
    }

    return [nodes, edges];
  }

  render() {
    /*const layout = {
            name: 'dagre',
            animate: true,
            rankDir: 'TB',
            ranker: 'longest-path' //tight-tree
        }*/
    //const layout = { name: 'breadthfirst' };
    //const layout = { name: 'circle' }
    const layout = {
      name: "klay",
      klay: {
        spacing: 40,
        direction: "DOWN",
        nodePlacement: "BRANDES_KOEPF",
        nodeLayering: "INTERACTIVE",
        fixedAlignment: "BALANCED",
        layoutHierarchy: true,
        mergeHierarchyCrossingEdges: false,
      },
    };

    cytoscape.use(klay);
    //cytoscape.use(cola);
    //cytoscape.use(dagre);

    let graph = this.prepare_graph(this.props.fsm_data);
    let nodes = graph[0];
    let edges = graph[1];

    return (
      <div
        style={{
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
        }}
      >
        <Grid container spacing={1}>
          <Grid item xs={12}>
            <Typography variant="h4" component="h4" gutterBottom align="center">
              {this.props.fsm_data.fsm_name}
            </Typography>
          </Grid>

          <Grid item xs={12}>
            <Box
              display="flex"
              border={1}
              justifyContent="center"
              style={{ width: "100%", height: "100%" }}
            >
              <CytoscapeComponent
                elements={CytoscapeComponent.normalizeElements({
                  nodes: nodes,
                  edges: edges,
                })}
                stylesheet={[
                  {
                    selector: "node",
                    style: {
                      label: "data(label)",
                      borderColor: "black",
                      borderWidth: 2,
                      textValign: "center",
                      textHalign: "center",
                      width: "data(width)",
                      height: "data(height)",
                    },
                  },
                  {
                    selector: "node[type = 'fsm']",
                    style: {
                      textValign: "top",
                      textHalign: "center",
                    },
                  },
                  {
                    selector: "node[type = 'outcome']",
                    style: {
                      backgroundColor: "red",
                      shape: "round-rectangle",
                    },
                  },
                  {
                    selector: "node[type = 'current_state']",
                    style: {
                      backgroundColor: "green",
                    },
                  },
                  {
                    selector: "node[type = 'current_fsm']",
                    style: {
                      borderColor: "blue",
                      textValign: "top",
                      textHalign: "center",
                    },
                  },
                  {
                    selector: "edge",
                    style: {
                      label: "data(label)",
                      targetArrowShape: "triangle",
                      curveStyle: "bezier", //unbundled
                      //"text-rotation": "autorotate",
                    },
                  },
                ]}
                layout={layout}
                style={{ width: "100%", height: "40vh" }}
                zoomingEnabled={true}
                boxSelectionEnabled={false}
                autoungrabify={true}
                panningEnabled={true}
                userZoomingEnabled={false}
                userPanningEnabled={false}
              />
            </Box>
          </Grid>
        </Grid>
      </div>
    );
  }
}

export default FSM;
