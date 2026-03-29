
"use strict";

let GetWaypointByName = require('./GetWaypointByName.js')
let AddNewWaypoint = require('./AddNewWaypoint.js')
let GetNumOfWaypoints = require('./GetNumOfWaypoints.js')
let SaveWaypoints = require('./SaveWaypoints.js')
let GetChargerByName = require('./GetChargerByName.js')
let GetWaypointByIndex = require('./GetWaypointByIndex.js')

module.exports = {
  GetWaypointByName: GetWaypointByName,
  AddNewWaypoint: AddNewWaypoint,
  GetNumOfWaypoints: GetNumOfWaypoints,
  SaveWaypoints: SaveWaypoints,
  GetChargerByName: GetChargerByName,
  GetWaypointByIndex: GetWaypointByIndex,
};
