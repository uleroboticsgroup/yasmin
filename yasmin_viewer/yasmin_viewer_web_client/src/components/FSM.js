import React, { useEffect, useMemo, useRef } from "react";
import CytoscapeComponent from "react-cytoscapejs";
import cytoscape from "cytoscape";
import klay from "cytoscape-klay";
import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";
import Grid from "@mui/material/Grid";

cytoscape.use(klay);

const Graph = React.memo(({ nodes, edges, layout, height }) => {
  const cyRef = useRef(null);

  // Updates only the `current_state` node's appearance
  useEffect(() => {
    if (cyRef.current) {
      cyRef.current.elements().removeClass("current_state");
      const currentNode = cyRef.current.$("node[type = 'current_state']");
      if (currentNode) currentNode.addClass("current_state");
    }
  }, [nodes]);

  return (
    <CytoscapeComponent
      cy={(cy) => (cyRef.current = cy)}
      elements={CytoscapeComponent.normalizeElements({ nodes, edges })}
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
            events: "no",
          },
        },
        {
          selector: "node[type = 'hidden_fsm']",
          style: {
            shape: "octagon",
            borderWidth: 3,
          },
        },
        {
          selector: "node[type = 'current_hidden_fsm']",
          style: {
            shape: "octagon",
            borderWidth: 3,
            backgroundColor: "green",
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
            curveStyle: "bezier",
            loopDirection: "-30deg",
            loopSweep: "-30deg",
            events: "no",
          },
        },
      ]}
      layout={layout}
      style={{ width: "100%", height }}
      zoomingEnabled={true}
      boxSelectionEnabled={false}
      autoungrabify={true}
      panningEnabled={true}
      userZoomingEnabled={true}
      userPanningEnabled={true}
    />
  );
});

const FSM = React.memo(({ fsm_data, alone, hide_nested_fsm }) => {
  const layout = useMemo(
    () => ({
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
    }),
    []
  );

  const height = alone ? "80vh" : "40vh";

  // Memoize nodes and edges to avoid recomputation unless `fsm_data` changes
  const { nodes, edges } = useMemo(() => {
    if (!fsm_data || fsm_data.length === 0) return { nodes: [], edges: [] }; // Guard clause

    const prepare_graph = (fsm_data, hide_nested_fsm) => {
      let nodes = [];
      let edges = [];
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

        if (state.is_fsm && !hide_nested_fsm) {
          type = "fsm";
        } else {
          if (current_state === state.id) {
            if (state.is_fsm && hide_nested_fsm) {
              type = "current_hidden_fsm";
            } else {
              type = "current_state";
            }
          } else if (state.is_fsm && hide_nested_fsm) {
            type = "hidden_fsm";
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

        for (let outcome_id in state.outcomes) {
          let outcome = state.outcomes[outcome_id];
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

          let target = 0;
          let source = state.id;

          if (state.is_fsm && !hide_nested_fsm) {
            source = source + outcome;
          }

          if (state.transitions.hasOwnProperty(outcome)) {
            let transition = state.transitions[outcome];

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

            if (target === 0 && state.parent >= 0) {
              for (let outcome_id in fsm_data[state.parent].outcomes) {
                let outcome = fsm_data[state.parent].outcomes[outcome_id];
                if (outcome === transition) {
                  target = state.parent + outcome;
                  break;
                }
              }
            }
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

      return { nodes, edges };
    };

    return prepare_graph(fsm_data, hide_nested_fsm);
  }, [fsm_data, hide_nested_fsm]);

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
            {fsm_data && fsm_data[0]?.name}
          </Typography>
        </Grid>

        <Grid item xs={12}>
          <Box
            display="flex"
            border={5}
            justifyContent="center"
            style={{ width: "100%", height: "100%" }}
          >
            <Graph
              nodes={nodes}
              edges={edges}
              layout={layout}
              height={height}
            />
          </Box>
        </Grid>
      </Grid>
    </div>
  );
});

export default FSM;
