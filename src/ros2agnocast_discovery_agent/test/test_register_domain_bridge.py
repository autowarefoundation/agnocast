"""Tests for the standalone domain bridge rule injector."""
import pytest

from ros2agnocast_discovery_agent import register_domain_bridge
from ros2agnocast_discovery_agent.domain_bridge_config import CONFIG_ENV


class FakeAddRule:
    """Stand-in for the ioctl wrapper symbol: records calls, returns a code per topic."""

    def __init__(self, codes=None):
        self.calls = []
        self._codes = codes or {}

    def __call__(self, from_topic, to_topic, from_domain, to_domain):
        from_name = from_topic.decode('utf-8')
        to_name = to_topic.decode('utf-8')
        self.calls.append((from_name, to_name, from_domain, to_domain))
        return self._codes.get(from_name, 0)


def _write_config(tmp_path, text):
    path = tmp_path / 'bridge.yaml'
    path.write_text(text)
    return str(path)


def test_no_config_is_usage_error(monkeypatch):
    monkeypatch.delenv(CONFIG_ENV, raising=False)
    with pytest.raises(SystemExit):
        register_domain_bridge.main([])


def test_registers_every_rule(tmp_path, monkeypatch):
    fake = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    cfg = _write_config(tmp_path, 'from_domain: 1\nto_domain: 2\ntopics:\n  chatter:\n')
    assert register_domain_bridge.main(['--config', cfg]) == 0
    assert fake.calls == [('chatter', 'chatter', 1, 2)]


def test_registers_remapped_rule_with_both_names(tmp_path, monkeypatch):
    fake = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    cfg = _write_config(
        tmp_path,
        'from_domain: 1\nto_domain: 2\ntopics:\n  /in_sub/chatter:\n    remap: /chatter\n')
    assert register_domain_bridge.main(['--config', cfg]) == 0
    assert fake.calls == [('/in_sub/chatter', '/chatter', 1, 2)]


def test_returns_nonzero_when_a_rule_is_rejected(tmp_path, monkeypatch):
    fake = FakeAddRule(codes={'chatter': -16})  # -EBUSY: an endpoint already exists
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    cfg = _write_config(tmp_path, 'from_domain: 1\nto_domain: 2\ntopics:\n  chatter:\n')
    assert register_domain_bridge.main(['--config', cfg]) == 1


def test_skipped_topic_is_reported(tmp_path, monkeypatch, capsys):
    fake = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    # No domains resolve for 'chatter', so it is skipped rather than registered.
    cfg = _write_config(tmp_path, 'topics:\n  chatter:\n')
    assert register_domain_bridge.main(['--config', cfg]) == 0
    assert fake.calls == []
    assert 'skipping chatter' in capsys.readouterr().err


def test_returns_nonzero_on_unreadable_config(tmp_path, monkeypatch):
    monkeypatch.setattr(
        register_domain_bridge, '_load_add_rule_symbol',
        lambda: pytest.fail('the wrapper must not load when the config is unreadable'))
    missing = str(tmp_path / 'does_not_exist.yaml')
    assert register_domain_bridge.main(['--config', missing]) == 1


def test_config_path_falls_back_to_env(tmp_path, monkeypatch):
    fake = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    cfg = _write_config(tmp_path, 'from_domain: 3\nto_domain: 4\ntopics:\n  image:\n')
    monkeypatch.setenv(CONFIG_ENV, cfg)
    assert register_domain_bridge.main([]) == 0
    assert fake.calls == [('image', 'image', 3, 4)]
