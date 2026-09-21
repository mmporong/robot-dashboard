/* Pure presentation logic tests: no ROS, browser, or robot connection. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const test = require('node:test');
const source = fs.readFileSync(`${__dirname}/dashboard.js`, 'utf8');
const context = {};
vm.runInNewContext(source.slice(0, source.indexOf('  loadPrefs();')) +
  '\n globalThis.ui={nodeRole,topicRole,graphRows};})();', context);
const {nodeRole, topicRole, graphRows} = context.ui;

test('configured nodes and topics have concise roles without health claims', () => {
  assert.match(nodeRole('/camera/camera'), /RGB/);
  assert.match(nodeRole('/ns/collision_monitor'), /정지/);
  assert.match(nodeRole('/_ros2cli_daemon_12_abc'), /검색/);
  assert.match(topicRole('/camera/depth/camera_info'), /보정/);
  assert.match(topicRole('/velocity_smoother/transition_event'), /전환/);
  assert.match(topicRole('/cmd_vel'), /최종/);
  assert.match(nodeRole('/unknown_node'), /미등록/);
  assert.match(topicRole('/unknown_topic'), /미등록/);
});
test('same fully qualified name is grouped with count, namespaces stay distinct', () => {
  const rows = graphRows({live:true,node_names:['/a/camera','/a/camera','/b/camera'],topic_names:[]});
  assert.equal(rows.nodes.length, 2);
  assert.equal(rows.nodes[0].count, 2);
  assert.equal(rows.nodes[1].count, 1);
});
test('role and path search is case insensitive and does not alter totals', () => {
  const graph={live:true,node_names:['/camera/camera','/collision_monitor'],topic_names:['/camera/color/image_raw','/scan']};
  assert.equal(graphRows(graph,'RGB').nodes.length, 1);
  assert.equal(graphRows(graph,'CAMERA').topics.length, 1);
  assert.equal(graphRows(graph,'라이다').topics.length, 1);
  assert.equal(graph.node_names.length, 2);
});
test('stale graph hides all rows and does not retain healthy observations', () => {
  const rows=graphRows({live:false,node_names:['/camera/camera'],topic_names:['/scan']});
  assert.equal(rows.nodes.length,0);
  assert.equal(rows.topics.length,0);
});
