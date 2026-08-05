import pytest

from newton_worker import protocol


def test_msgpack_roundtrip():
    if not protocol.HAVE_MSGPACK:
        pytest.skip("msgpack not installed")
    message = {"id": 7, "op": "step", "params": {"ctrl": [0.5, -1.0], "blob": b"\x00\x01"}}
    decoded, codec = protocol.decode(protocol.encode(message, protocol.CODEC_MSGPACK))
    assert codec == protocol.CODEC_MSGPACK
    assert decoded == message


def test_json_roundtrip_and_sniffing():
    message = {"id": 1, "op": "hello", "params": {"protocol": 1}}
    decoded, codec = protocol.decode(protocol.encode(message, protocol.CODEC_JSON))
    assert codec == protocol.CODEC_JSON
    assert decoded == message


def test_decode_garbage_raises():
    with pytest.raises(ValueError):
        protocol.decode(b"\xff\xfe not a message")


def test_validate_request():
    assert protocol.validate_request({"id": 3, "op": "step"}) == (3, "step", {})
    with pytest.raises(ValueError):
        protocol.validate_request({"op": "step"})  # missing id
    with pytest.raises(ValueError):
        protocol.validate_request({"id": 1})  # missing op
    with pytest.raises(ValueError):
        protocol.validate_request({"id": 1, "op": "step", "params": [1]})  # bad params


def test_reply_shapes():
    ok = protocol.ok_reply(5, {"a": 1})
    assert ok == {"id": 5, "ok": True, "result": {"a": 1}}
    err = protocol.error_reply(5, protocol.ERR_NO_MODEL, "nope")
    assert err["ok"] is False and err["error"]["code"] == "no_model"
