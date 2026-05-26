"""Unit tests for pure verb helpers."""

from ros2agnocast.verb.node_info_agnocast import (
    service_name_from_request_topic,
    service_name_from_response_topic,
)


def test_service_name_from_request_topic_strips_prefix():
    assert service_name_from_request_topic(
        '/AGNOCAST_SRV_REQUEST/add_two_ints') == '/add_two_ints'


def test_service_name_from_request_topic_returns_none_for_non_service():
    assert service_name_from_request_topic('/regular_topic') is None


def test_service_name_from_response_topic_strips_prefix_and_sep_suffix():
    assert service_name_from_response_topic(
        '/AGNOCAST_SRV_RESPONSE/add_two_ints_SEP_42') == '/add_two_ints'


def test_service_name_from_response_topic_returns_none_for_non_service():
    assert service_name_from_response_topic('/regular_topic') is None
