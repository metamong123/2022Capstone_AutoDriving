/*
@licstart  The following is the entire license notice for the
JavaScript code in this file.

Copyright (C) 1997-2019 by Dimitri van Heesch

This program is free software; you can redistribute it and/or modify
it under the terms of version 2 of the GNU General Public License as published by
the Free Software Foundation

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

@licend  The above is the entire license notice
for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "Xsens Device API", "index.html", [
    [ "Xsens Device API Library", "index.html", [
      [ "Introduction", "index.html#intro_xda", null ],
      [ "Introduction to the API", "index.html#s_overview", null ],
      [ "Example Code", "index.html#s_examples", null ],
      [ "ROS MTi driver", "index.html#s_rosdriver", null ]
    ] ],
    [ "Upgrading from XDA 4.x", "migration.html", [
      [ "Upgrading from XDA 4.0 to XDA 4.1 or higher", "migration.html#mig_40_41_intro", [
        [ "Breaking changes", "migration.html#mig_40_41_changes", null ],
        [ "New features", "migration.html#mig_40_41_new", null ]
      ] ],
      [ "Upgrading from XDA 4.4 to XDA 4.5 or higher", "migration.html#mig_44_45_intro", [
        [ "Breaking changes", "migration.html#mig_44_45_changes", null ],
        [ "New features", "migration.html#mig_44_45_new", null ]
      ] ],
      [ "Upgrading from XDA 4.6 to XDA 4.7 or higher", "migration.html#mig_46_47_intro", [
        [ "Breaking changes", "migration.html#mig_46_47_changes", null ],
        [ "New features", "migration.html#mig_46_47_new", null ]
      ] ],
      [ "Upgrading from XDA 4.7 to XDA 4.8.6 or higher", "migration.html#mig_47_50_intro", [
        [ "Breaking changes", "migration.html#mig_47_50_changes", null ],
        [ "New features", "migration.html#mig_47_50_new", null ]
      ] ]
    ] ],
    [ "Overview of the Xsens Device API", "overview.html", [
      [ "Introduction", "overview.html#s_introduction", null ],
      [ "Set-up", "overview.html#s_setup", [
        [ "Important Classes of the C++ interface", "overview.html#ss_mainClasses", null ],
        [ "Device Hierarchy", "overview.html#ss_deviceTree", null ]
      ] ],
      [ "Device Diversity", "overview.html#s_deviceDiversity", [
        [ "Program Flow", "overview.html#ss_programFlow", null ],
        [ "Concurrency", "overview.html#s_concurrency", null ]
      ] ]
    ] ],
    [ "Xsens Device API Objectives and Considerations", "p_background.html", [
      [ "Objectives", "p_background.html#s_objectives", null ],
      [ "Considerations", "p_background.html#s_considerations", [
        [ "Language", "p_background.html#ss_language", null ],
        [ "Name mangling", "p_background.html#ss_nameMangling", null ],
        [ "Generic types libraries", "p_background.html#ss_genericTypes", null ],
        [ "Generic, complexity and flexibility", "p_background.html#ss_generic", null ],
        [ "Used internally", "p_background.html#ss_usedInternally", null ]
      ] ]
    ] ],
    [ "Code Examples", "p_examples.html", [
      [ "XDA MTi Examples", "p_examples.html#s_xda_mti_ex", null ],
      [ "XDA MTw Examples", "p_examples.html#s_xda_mtw_ex", null ],
      [ "Built Examples", "p_examples.html#s_xda_built_ex", null ],
      [ "Embedded Examples", "p_examples.html#s_xda_embedded_ex", null ],
      [ "Questions?", "p_examples.html#s_question_ex", null ]
    ] ],
    [ "Workflow using the Xsens Awinda System", "awindaworkflow.html", [
      [ "Introduction", "awindaworkflow.html#s_intro", null ],
      [ "Awinda Master states", "awindaworkflow.html#s_awsstates", null ],
      [ "Mtw states", "awindaworkflow.html#s_mtwstates", null ],
      [ "Work Flow", "awindaworkflow.html#s_workflow", null ]
    ] ],
    [ "Public Xsens Device API", "p_publicxda.html", [
      [ "Introduction", "p_publicxda.html#s_publicXdaIntroduction", null ],
      [ "Overview", "p_publicxda.html#s_overview_pxda", null ],
      [ "Requirements", "p_publicxda.html#s_requirements_pxda", null ],
      [ "Getting started", "p_publicxda.html#s_gettingstarted_pxda", null ],
      [ "Questions?", "p_publicxda.html#s_question_pxda", null ]
    ] ],
    [ "Retransmissions and Stream behaviour for Xsens Awinda Systems", "awindaretransmissions.html", [
      [ "Summary", "awindaretransmissions.html#awindaretransmissions_summary", [
        [ "The Live Stream", "awindaretransmissions.html#awindaretransmissions_livestream", null ],
        [ "The Buffered Stream", "awindaretransmissions.html#awindaretransmissions_bufferedstream", null ],
        [ "Flushing Mode", "awindaretransmissions.html#awindaretransmissions_flushingmode", null ],
        [ "The Data Stream", "awindaretransmissions.html#awindaretransmissions_datastream", null ]
      ] ],
      [ "Multiple MTw behaviour", "awindaretransmissions.html#awindaretransmissions_multidev", [
        [ "The All Live Stream", "awindaretransmissions.html#awindaretransmissions_alllivestream", null ],
        [ "The All Buffered Stream", "awindaretransmissions.html#awindaretransmissions_allbufferedstream", null ],
        [ "The All Data Stream", "awindaretransmissions.html#awindaretransmissions_alldatastream", null ]
      ] ],
      [ "A note on delays in the Buffered Stream", "awindaretransmissions.html#awindaretransmissions_datastreamdelays", null ]
    ] ],
    [ "Xsens MTi driver for ROS", "p_rosdriver.html", [
      [ "Introduction", "p_rosdriver.html#s_intrt_rosdriver", null ],
      [ "Overview", "p_rosdriver.html#s_overview_rosdriver", null ],
      [ "Requirements", "p_rosdriver.html#s_requirements_rosdriver", null ],
      [ "Building the Xsens MTi node", "p_rosdriver.html#s_building_rosdriver", null ],
      [ "Running the Xsens MTi node", "p_rosdriver.html#s_running_rosdriver", null ],
      [ "Configuring the Xsens MTi node", "p_rosdriver.html#s_configuring_rosdriver", [
        [ "Published topics", "p_rosdriver.html#subs_topics_rosdriver", null ]
      ] ],
      [ "Troubleshooting", "p_rosdriver.html#s_troubleshooting_rosdriver", null ],
      [ "Questions?", "p_rosdriver.html#s_question_rosdriver", null ]
    ] ],
    [ "Deprecated List", "deprecated.html", null ],
    [ "Modules", "modules.html", "modules" ],
    [ "Namespace Members", "namespacemembers.html", [
      [ "All", "namespacemembers.html", null ],
      [ "Functions", "namespacemembers_func.html", null ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", "functions_func" ],
        [ "Variables", "functions_vars.html", "functions_vars" ],
        [ "Typedefs", "functions_type.html", null ],
        [ "Related Functions", "functions_rela.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"annotated.html",
"group___callbacks.html#gaf94a5a76c3a443255fc53f7a2bf9e076",
"group__cinterface.html#ga42d5a40697d57c0da08e1d21c82fb6e6",
"group__cinterface.html#ga7b75e01c27f76b4d91aaa135e4e1a5c3",
"group__cinterface.html#gabc9ccab13c489003637dc63769765d3a",
"group__cinterface.html#gafd92f0315fa77840940cd63b25612f52",
"group__enums.html#gga760018646ba5d22bd56b996d1c715424ae24a4881460c18909206460d43d362d8",
"group__functions.html#gad8bc078dcde53a9ba7e86cdccfc926e4",
"struct_xs_control.html#a0281f1055d6080734ce67526e2210fd3",
"struct_xs_device.html#a208157faba015c166f2878e070cfad94",
"struct_xs_device_id.html#a36e956e5e27fe4144a4b703d3049aefb",
"struct_xs_filter_profile_array.html#af9d20131a77f037d4ea148bfa7c8373a",
"struct_xs_port_info_array.html#a0cac36bd34d41127a9209aaf5dd27343",
"struct_xs_string.html#a3fd882a68840130cd7f8f409a1c30ab0",
"struct_xs_version.html#af8b0517592573d999edda0dd3b70091c"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';