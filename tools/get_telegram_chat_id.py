#!/usr/bin/env python3
"""Prints recent Telegram updates and extracts chat ids.

Usage:
  BOT_TOKEN env var or TELEGRAM_BOT_TOKEN env var is used.
  python tools/get_telegram_chat_id.py
"""
import os
import sys
import json
try:
    # Python 3
    from urllib.request import urlopen, Request
    from urllib.parse import urlencode
except Exception:
    sys.exit("Unsupported Python")

def get_token():
    return os.environ.get("TELEGRAM_BOT_TOKEN") or os.environ.get("BOT_TOKEN")

def get_updates(token):
    url = f"https://api.telegram.org/bot{token}/getUpdates"
    req = Request(url)
    with urlopen(req, timeout=10) as r:
        return json.load(r)

def extract_chat_info(updates):
    out = []
    for item in updates.get("result", []):
        msg = item.get("message") or item.get("edited_message") or item.get("channel_post") or item.get("my_chat_member")
        if not msg:
            continue
        chat = msg.get("chat", {})
        cid = chat.get("id")
        ctype = chat.get("type")
        title = chat.get("title") or chat.get("username") or chat.get("first_name") or ""
        out.append({"id": cid, "type": ctype, "title": title, "raw": chat})
    return out

def main():
    token = get_token()
    if not token:
        print("ERROR: set TELEGRAM_BOT_TOKEN or BOT_TOKEN in your environment")
        sys.exit(2)
    try:
        updates = get_updates(token)
    except Exception as e:
        print("ERROR: failed to call getUpdates:", e)
        sys.exit(3)
    print(json.dumps(updates, indent=2))
    infos = extract_chat_info(updates)
    if not infos:
        print("No chats found in updates. Send a message to the bot or post in the group and try again.")
        return
    print("\nExtracted chats:")
    for i in infos:
        print(f"{i['id']}  {i['type']}  {i['title']}")

if __name__ == '__main__':
    main()
