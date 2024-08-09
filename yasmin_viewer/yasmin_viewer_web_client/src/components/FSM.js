// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

import React from "react";

import CytoscapeComponent from "react-cytoscapejs";
import cytoscape from "cytoscape";
import klay from "cytoscape-klay";
//import cola from "cytoscape-cola";
//import dagre from "cytoscape-dagre";

import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";
import Grid from "@mui/material/Grid";

class FSM extends React.Component {
  prepare_graph(fsm_data, hide_nested_fsm) {
    let nodes = [];
    let edges = [];

    // get current state
    let current_state = 0;

    if (!hide_nested_fsm) {
      while (current_state >= 0) {
        if (fsm_data[current_state].is_fsm) {
          current_state = fsm_data[current_state].current_state;
        } else {
          break;
        }
      }
    } else {
      current_state = fsm_data[0].current_state;
    }

    // create nodes and edges
    for (let state_id in fsm_data) {
      let state = fsm_data[state_id];
      let type = "state";
      let name = state.name;

      if (state.id === 0) {
        name = "";
      }

      if (hide_nested_fsm && state.parent !== 0 && state.id !== 0) {
        continue;
      }

      // current state
      if (state.is_fsm && !hide_nested_fsm) {
        type = "fsm";
      } else {
        if (current_state === state.id) {
          type = "current_state";
        }
      }

      if (state.id > 0) {
        nodes.push({
          data: {
            id: fsm_data[0].name + "node" + state.id,
            parent: fsm_data[0].name + "node" + state.parent,
            label: name,
            type: type,
          },
        });
      }

      // outcome
      for (let outcome_id in state.outcomes) {
        let outcome = state.outcomes[outcome_id];

        // FSM outcome
        if (state.is_fsm) {
          if (!hide_nested_fsm || (hide_nested_fsm && state.id === 0)) {
            nodes.push({
              data: {
                id: fsm_data[0].name + "node" + state.id + outcome,
                parent: fsm_data[0].name + "node" + state.id,
                label: outcome,
                type: "outcome",
              },
            });
          }
        }

        // edges
        let target = 0;
        let source = state.id;

        if (state.is_fsm && !hide_nested_fsm) {
          source = source + outcome;
        }

        if (state.transitions.hasOwnProperty(outcome)) {
          let transition = state.transitions[outcome];

          // transition to state
          for (let aux_state_id in fsm_data) {
            let aux_state = fsm_data[aux_state_id];

            if (
              aux_state.name === transition &&
              aux_state.parent === state.parent
            ) {
              target = aux_state.id;
              break;
            }
          }

          // transition to outcome
          if (target === 0 && state.parent >= 0) {
            for (let outcome_id in fsm_data[state.parent].outcomes) {
              let outcome = fsm_data[state.parent].outcomes[outcome_id];
              if (outcome === transition) {
                target = state.parent + outcome;
                break;
              }
            }
          }

          // state outcome to parent outcome
        } else {
          target = state.parent + outcome;
        }

        if (state.parent >= 0) {
          edges.push({
            data: {
              id: fsm_data[0].name + "edge" + state.id + outcome,
              source: fsm_data[0].name + "node" + source,
              target: fsm_data[0].name + "node" + target,
              label: outcome,
            },
          });
        }
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
        nodeLayering: "LONGEST_PATH",
        fixedAlignment: "BALANCED",
        layoutHierarchy: true,
        mergeHierarchyCrossingEdges: false,
      },
    };

    cytoscape.use(klay);
    //cytoscape.use(cola);
    //cytoscape.use(dagre);

    if (this.props.fsm_data === undefined) {
      return <div></div>;
    }

    let graph = this.prepare_graph(
      this.props.fsm_data,
      this.props.hide_nested_fsm
    );

    let nodes = graph[0];
    let edges = graph[1];

    let height = "40vh";
    if (this.props.alone) {
      height = "80vh";
    }

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
              {this.props.fsm_data[0].name}
            </Typography>
          </Grid>

          <Grid item xs={12}>
            <Box
              display="flex"
              border={5}
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
                      fontSize: 15,
                      height: "label",
                      width: "label",
                      paddingBottom: 15,
                      paddingLeft: 20,
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
                      paddingTop: 10,
                      paddingLeft: 10,
                    },
                  },
                  {
                    selector: "node[type = 'current_state']",
                    style: {
                      backgroundColor: "green",
                    },
                  },
                  {
                    selector: "edge",
                    style: {
                      label: "data(label)",
                      targetArrowShape: "triangle",
                      curveStyle: "bezier", //unbundled
                      //"text-rotation": "autorotate",
                      loopDirection: "-30deg",
                      loopSweep: "-30deg",
                    },
                  },
                ]}
                layout={layout}
                style={{ width: "100%", height: height }}
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
