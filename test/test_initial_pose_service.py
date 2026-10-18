"""Regression checks for asynchronous Cartographer initialization without DDS."""
from array import array
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped

from outdoors_loc_nav.util.initial_pose_serv import InitialPosePub


@pytest.fixture
def initializer():
    node = SimpleNamespace()
    for name, method in InitialPosePub.__dict__.items():
        if callable(method) and name != '__init__':
            setattr(node, name, MethodType(method, node))
    node.get_logger = Mock(return_value=Mock())
    node.state = 'TRAJ_QUERY'
    node.trajectory_states_client = Mock()
    node.finish_client = Mock()
    node.start_client = Mock()
    node.active_trajectories = [0, 1]
    node._finish_index = 0
    return node


@pytest.mark.parametrize('state,client', [
    ('TRAJ_QUERY', 'trajectory_states_client'),
    ('TRAJ_FINISH', 'finish_client'),
    ('TRAJ_START', 'start_client'),
])
def test_timer_waits_for_response(initializer, state, client):
    initializer.state = state
    initializer.config_dir = '/tmp'
    initializer.config_base = 'test.lua'
    initializer._build_pose = lambda: (0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    for _ in range(4):
        initializer.timer_callback()
    getattr(initializer, client).call_async.assert_called_once()


@pytest.mark.parametrize('callback', [
    '_on_get_states_done', '_on_finish_done', '_on_start_done',
])
@pytest.mark.parametrize('failure', ['status', 'exception', 'empty'])
def test_failed_service_retries_without_advancing(initializer, callback, failure):
    future = Mock()
    if failure == 'exception':
        future.result.side_effect = RuntimeError('transport failed')
    else:
        future.result.return_value = (
            SimpleNamespace(status=SimpleNamespace(code=3, message='invalid request'))
            if failure == 'status' else None)
    getattr(initializer, callback)(future)
    assert initializer.state == 'SERV_WAIT'
    assert initializer._finish_index == 0
    initializer.start_client.call_async.assert_not_called()


def test_rviz_pose_accepts_ros_covariance_array(initializer):
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.orientation.w = 1.0
    pose.pose.covariance = array('d', [1.0] + [0.0] * 35)
    initializer.latest_pose_msg = pose
    assert initializer._build_pose() == (0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_idle_timer_cannot_interrupt_pending_service(initializer):
    initializer.latest_odom_x = None
    initializer.latest_odom_y = None
    timer = Mock()
    initializer._idle_timer = timer
    initializer.destroy_timer = Mock()
    initializer._idle_timeout_callback()
    timer.cancel.assert_called_once()
    initializer.destroy_timer.assert_called_once_with(timer)
    assert initializer._idle_timer is None
