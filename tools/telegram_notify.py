#!/usr/bin/env python3
"""Send a Telegram message using BOT_TOKEN/TELEGRAM_BOT_TOKEN or BOT_TOKEN env var.

Usage:
  TELEGRAM_BOT_TOKEN or BOT_TOKEN and TELEGRAM_CHAT_ID env vars must be set, or pass chat id as first arg.
  python tools/telegram_notify.py "Message text"
"""
import os
import sys
import json
try:
    from urllib.request import urlopen, Request
    from urllib.parse import urlencode
except Exception:
    sys.exit("Unsupported Python")

def get_token():
    return os.environ.get("TELEGRAM_BOT_TOKEN") or os.environ.get("BOT_TOKEN")

def get_chat():
    return os.environ.get("TELEGRAM_CHAT_ID") or (sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].startswith("-") or (len(sys.argv)>1 and sys.argv[1].isdigit()) else None)

def send_message(token, chat_id, text):
    url = f"https://api.telegram.org/bot{token}/sendMessage"
    data = urlencode({"chat_id": str(chat_id), "text": text}).encode()
    req = Request(url, data=data, headers={"Content-Type": "application/x-www-form-urlencoded"})
    with urlopen(req, timeout=10) as r:
        return json.load(r)

def main():
    token = get_token()
    if not token:
        print("ERROR: set TELEGRAM_BOT_TOKEN or BOT_TOKEN in your environment")
        sys.exit(2)
    # message text may be args after optional chat id
    if os.environ.get("TELEGRAM_CHAT_ID"):
        chat = os.environ.get("TELEGRAM_CHAT_ID")
        msg = " ".join(sys.argv[1:]) or "Hello from VS Code"
    else:
        # if first arg is numeric (chat id), use it
        if len(sys.argv) >= 2 and (sys.argv[1].lstrip('-').isdigit()):
            chat = sys.argv[1]
            msg = " ".join(sys.argv[2:]) or "Hello from VS Code"
        else:
            print("ERROR: set TELEGRAM_CHAT_ID env var or pass chat id as first arg")
            sys.exit(2)
    try:
        res = send_message(token, chat, msg)
    except Exception as e:
        print("ERROR: send failed:", e)
        sys.exit(3)
    print(json.dumps(res, indent=2))

if __name__ == '__main__':
    main()
