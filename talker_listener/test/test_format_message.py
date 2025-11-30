import pytest

from talker_listener.talker import format_message


def test_format_message_basic():
    assert format_message("hello SSK", 0) == "hello SSK 0"
    assert format_message("prefix", 42) == "prefix 42"


@pytest.mark.parametrize("prefix,idx,expected", [
    ("a", 1, "a 1"),
    ("greetings", 5, "greetings 5"),
    ("", 0, " 0"),
])
def test_format_message_param(prefix, idx, expected):
    assert format_message(prefix, idx) == expected
