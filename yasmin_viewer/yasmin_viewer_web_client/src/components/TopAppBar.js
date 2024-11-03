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

import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Box from "@mui/material/Box";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import TextField from "@mui/material/TextField";
import Autocomplete from "@mui/material/Autocomplete";
import Checkbox from "@mui/material/Checkbox";
import FormControlLabel from "@mui/material/FormControlLabel";
import Divider from "@mui/material/Divider";

export default function TopAppBar({
  fsm_name_list,
  current_fsm,
  handle_current_fsm,
  handle_hide_nested_fsm,
  handle_show_only_active_fsms,
}) {
  const [currentFSM, setCurrentFSM] = React.useState("ALL");
  const [hideNestedFSM, setHideNestedFSM] = React.useState(false);
  const [showOnlyActiveFSMs, setShowOnlyActiveFSMs] = React.useState(false);

  const handleChangeHideNestedFSM = (event) => {
    setHideNestedFSM(event.target.checked);
  };

  const handleShowOnlyActiveFSMs = (event) => {
    setShowOnlyActiveFSMs(event.target.checked);
  };

  React.useEffect(() => {
    handle_current_fsm(currentFSM);
  }, [currentFSM]);

  React.useEffect(() => {
    if (current_fsm !== currentFSM) {
      setCurrentFSM(current_fsm);
    }
  }, [current_fsm]);

  React.useEffect(() => {
    handle_hide_nested_fsm(hideNestedFSM);
  }, [hideNestedFSM]);

  React.useEffect(() => {
    handle_show_only_active_fsms(showOnlyActiveFSMs);
  }, [showOnlyActiveFSMs]);

  return (
    <Box sx={{ flexGrow: 1 }} style={{ width: "100%", height: "6.75vh" }}>
      <AppBar style={{ margin: 0 }}>
        <Toolbar
          sx={{ bgcolor: "#1e88e5", minHeight: 80, height: 80 }}
          variant="dense"
          disableGutters
        >
          <Typography
            variant="h5"
            component="div"
            style={{ marginLeft: 30 }}
            sx={{
              display: { xs: "none", md: "flex" },
              fontFamily: "monospace",
              fontWeight: 700,
              letterSpacing: ".2rem",
              color: "inherit",
              textDecoration: "none",
            }}
          >
            YASMIN Viewer
          </Typography>

          <Divider
            orientation="vertical"
            variant="middle"
            flexItem
            color="white"
            style={{ marginLeft: 30, marginRight: 30 }}
          />

          <Typography
            variant="h6"
            component="div"
            sx={{
              display: { xs: "none", md: "flex" },
              fontFamily: "monospace",
              fontWeight: 700,
              letterSpacing: ".2rem",
              color: "inherit",
              textDecoration: "none",
            }}
          >
            FSM:
          </Typography>

          <Autocomplete
            id="combo-box"
            disableClearable={true}
            options={fsm_name_list}
            getOptionLabel={(option) => option}
            isOptionEqualToValue={(option, value) => option === value}
            defaultValue={"ALL"}
            value={currentFSM}
            onChange={(event, value) => {
              setCurrentFSM(value);
            }}
            style={{
              width: 300,
              backgroundColor: "#d1e7fa",
            }}
            renderInput={(params) => (
              <TextField {...params} variant="outlined" />
            )}
          />

          <Divider
            orientation="vertical"
            variant="middle"
            flexItem
            color="white"
            style={{ marginLeft: 30, marginRight: 30 }}
          />

          <FormControlLabel
            label="Hide Nested FSMs"
            control={
              <Checkbox
                checked={hideNestedFSM}
                onChange={handleChangeHideNestedFSM}
                inputProps={{ "aria-label": "controlled" }}
                sx={{
                  color: "white",
                  "&.Mui-checked": {
                    color: "white",
                  },
                }}
              />
            }
          />

          <Divider
            orientation="vertical"
            variant="middle"
            flexItem
            color="white"
            style={{ marginLeft: 30, marginRight: 30 }}
          />

          <FormControlLabel
            label="Show Only Active FSMs"
            control={
              <Checkbox
                checked={showOnlyActiveFSMs}
                onChange={handleShowOnlyActiveFSMs}
                inputProps={{ "aria-label": "controlled" }}
                sx={{
                  color: "white",
                  "&.Mui-checked": {
                    color: "white",
                  },
                }}
              />
            }
          />
        </Toolbar>
      </AppBar>
    </Box>
  );
}
