#!/usr/bin/env python3
"""
Training data for the fine-tuned RV-2AJ assistant (rv2aj-qwen).

Plays the operator: thousands of spoken-style requests (mostly Turkish, some
English, some with speech-recognition mistakes, some following on from an
earlier turn) with the exact plan each one means, in the same message format
ai_node uses with compact=True. Every label is checked against the command
grammar and the validator's hard rules before it is written.

    python3 gen_dataset.py --n 6000 --seed 1 --out data/train.jsonl
"""

import argparse
import json
import os
import random
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from rv2aj_control import ai_brain as b  # noqa: E402

R = random.Random(1)


def pick(xs):
    return R.choice(xs)


def maybe(p):
    return R.random() < p


# -- Turkish numbers ---------------------------------------------------------------------

_ONES = ["", "bir", "iki", "üç", "dört", "beş", "altı", "yedi", "sekiz", "dokuz"]
_TENS = ["", "on", "yirmi", "otuz", "kırk", "elli", "altmış", "yetmiş", "seksen", "doksan"]


def tr_num(n):
    """Turkish words for 0 < n < 1000 (integers)."""
    n = int(n)
    parts = []
    if n >= 100:
        h = n // 100
        parts.append(("" if h == 1 else _ONES[h] + " ") + "yüz")
        n %= 100
    if n >= 10:
        parts.append(_TENS[n // 10])
        n %= 10
    if n:
        parts.append(_ONES[n])
    return " ".join(parts)


_EN_ONES = ["", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven", "twelve",
            "thirteen", "fourteen", "fifteen", "sixteen", "seventeen", "eighteen", "nineteen"]
_EN_TENS = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]


def en_num(n):
    n = int(n)
    if n < 20:
        return _EN_ONES[n]
    if n < 100:
        return _EN_TENS[n // 10] + ("" if n % 10 == 0 else " " + _EN_ONES[n % 10])
    return str(n)


def say_num(n, lang="tr", words_p=0.3):
    if float(n) != int(n):
        return f"{n:g}".replace(".", ",") if lang == "tr" else f"{n:g}"
    if maybe(words_p) and n < 200:
        return tr_num(n) if lang == "tr" else en_num(n)
    return str(int(n))


def last_vowel(word):
    for ch in reversed(word.lower()):
        if ch in "aıouâ":
            return "back"
        if ch in "eiöü":
            return "front"
    return "front"


_DIGIT_SOUND = {"0": "sıfır", "1": "bir", "2": "iki", "3": "üç", "4": "dört", "5": "beş", "6": "altı", "7": "yedi",
                "8": "sekiz", "9": "dokuz"}


def dative(name):
    """P3 -> P3'e, kutu -> kutuya, palet -> palete, P6 -> P6'ya."""
    spoken = _DIGIT_SOUND[name[-1]] if name[-1].isdigit() else name
    vowel_end = spoken[-1].lower() in "aeıioöuü"
    suffix = ("y" if vowel_end else "") + ("a" if last_vowel(spoken) == "back" else "e")
    return name + ("'" if name[0].isupper() or name[-1].isdigit() else "") + suffix


# -- amounts -----------------------------------------------------------------------------

def tr_amount_mm():
    """(spoken, mm) for a distance. None mm means "no amount said"."""
    r = R.random()
    if r < 0.45:
        cm = pick([1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 7, 8, 10, 10, 12, 15, 20])
        unit = pick(["santim", "santim", "santim", "cm", "santimetre"])
        return f"{say_num(cm)} {unit}", cm * 10.0
    if r < 0.62:
        mm = pick([5, 5, 10, 10, 15, 20, 25, 30, 40, 50, 60, 75, 80, 100, 120, 150])
        unit = pick(["milimetre", "milim", "mm"])
        return f"{say_num(mm)} {unit}", float(mm)
    if r < 0.68:
        cm = pick([1, 2, 3, 4, 5])
        return f"{tr_num(cm)} buçuk santim", cm * 10.0 + 5
    if r < 0.71:
        return "yarım santim", 5.0
    if r < 0.82:
        return pick(["biraz", "azıcık", "hafifçe", "biraz", "birazcık", "az"]), 10.0
    if r < 0.88:
        return pick(["çok", "epey", "bayağı", "baya"]), 50.0
    if r < 0.91:
        return "MAX", float(b.MAX_TCP_MM)
    return "", 20.0


def en_amount_mm():
    r = R.random()
    if r < 0.45:
        cm = pick([1, 2, 3, 4, 5, 6, 8, 10, 12, 15])
        return f"{say_num(cm, 'en')} {pick(['cm', 'centimeters', 'centimetres', 'cm'])}", cm * 10.0
    if r < 0.65:
        mm = pick([5, 10, 15, 20, 25, 30, 40, 50, 75, 100])
        return f"{say_num(mm, 'en')} {pick(['mm', 'millimeters', 'mm'])}", float(mm)
    if r < 0.8:
        return pick(["a bit", "a little", "slightly", "a touch"]), 10.0
    if r < 0.88:
        return pick(["a lot", "far"]), 50.0
    return "", 20.0


def tr_amount_deg():
    r = R.random()
    if r < 0.7:
        d = pick([5, 10, 10, 15, 20, 25, 30, 30, 45, 45, 60, 90])
        return f"{say_num(d)} derece", float(d)
    if r < 0.82:
        return pick(["biraz", "azıcık", "hafifçe"]), 5.0
    if r < 0.88:
        return pick(["çok", "epey"]), 30.0
    return "", 10.0


def en_amount_deg():
    r = R.random()
    if r < 0.75:
        d = pick([5, 10, 15, 20, 30, 45, 60, 90])
        return f"{say_num(d, 'en')} degrees", float(d)
    if r < 0.88:
        return pick(["a bit", "slightly", "a little"]), 5.0
    return "", 10.0


def mm_text(mm):
    return f"{mm / 10:g} santim".replace(".", ",") if mm >= 10 and mm % 5 == 0 else f"{mm:g} milimetre"


# -- phrase banks ------------------------------------------------------------------------

# (direction word forms, intransitive verb forms, transitive verb forms)
TR_DIRS = {
    "UP": (["yukarı", "yukarı", "yukarıya", "yukarı doğru"],
           ["çık", "git", "kalk", "yüksel", "çıkar mısın", "çıkabilir misin", "çıkalım", "çıksın", "gider misin",
            "hareket et", "çık lütfen", "kalkar mısın"],
           ["kaldır", "kaldırır mısın", "kaldıralım", "götür", "al", "kaldırabilir misin", "çıkar"]),
    "DOWN": (["aşağı", "aşağı", "aşağıya", "aşağı doğru"],
             ["in", "git", "alçal", "iner misin", "inebilir misin", "inelim", "insin", "gider misin", "hareket et",
              "in lütfen", "alçalır mısın"],
             ["indir", "indirir misin", "indirelim", "götür", "indirebilir misin", "al"]),
    "LEFT": (["sola", "sola", "sol tarafa", "sola doğru"],
             ["git", "kay", "gider misin", "gidebilir misin", "kayar mısın", "gidelim", "kaysın", "hareket et",
              "ilerle", "git lütfen"],
             ["kaydır", "götür", "kaydırır mısın", "çek", "al", "kaydıralım"]),
    "RIGHT": (["sağa", "sağa", "sağ tarafa", "sağa doğru"],
              ["git", "kay", "gider misin", "gidebilir misin", "kayar mısın", "gidelim", "kaysın", "hareket et",
               "ilerle", "git lütfen"],
              ["kaydır", "götür", "kaydırır mısın", "çek", "al", "kaydıralım"]),
    "FORWARD": (["ileri", "ileri", "ileriye", "öne", "öne doğru", "dışarı doğru"],
                ["git", "uzan", "ilerle", "gider misin", "uzanır mısın", "ilerler misin", "gidelim", "hareket et",
                 "git lütfen"],
                ["uzat", "götür", "uzatır mısın", "it", "al"]),
    "BACK": (["geri", "geriye", "arkaya", "içeri doğru", "tabana doğru", "geri"],
             ["git", "gel", "çekil", "gider misin", "gelir misin", "çekilir misin", "gidelim", "hareket et"],
             ["çek", "çeker misin", "götür", "çekelim", "al"]),
}
TR_OBJECTS = ["kolu", "robotu", "tutucuyu", "ucu", "aracı", "kolu", "gripperı"]
TR_PREFIX = ["", "", "", "", "hocam ", "lütfen ", "şimdi ", "tamam, ", "hadi ", "peki ", "robot, ", "şimdi de ",
             "tamam hocam, ", "evet, "]
TR_SUFFIX = ["", "", "", "", " lütfen", " hocam", ".", "!", " bakalım", " hemen"]

EN_DIRS = {
    "UP": (["up", "upward"], ["move", "go", "raise it", "lift it", "move it"], ["raise", "lift"]),
    "DOWN": (["down", "downward"], ["move", "go", "lower it", "move it", "drop"], ["lower", "descend"]),
    "LEFT": (["left", "to the left"], ["move", "go", "shift", "slide", "move it"], []),
    "RIGHT": (["right", "to the right"], ["move", "go", "shift", "slide", "move it"], []),
    "FORWARD": (["forward", "outward", "out"], ["move", "go", "reach", "extend", "push"], []),
    "BACK": (["back", "backward", "in toward the base"], ["move", "go", "pull", "retract", "come"], []),
}


def with_frame(text, lang="tr", question=False):
    if question and lang == "tr":
        return (pick(["", "", "", "hocam ", "peki ", "bir şey soracağım, ", "merak ettim, "]) + text +
                pick(["", "", "?", "?", " hocam", " hocam?"])).strip()
    if lang == "tr":
        return (pick(TR_PREFIX) + text + pick(TR_SUFFIX)).strip()
    return (pick(["", "", "", "please ", "ok, ", "now ", "robot, "]) + text + pick(["", "", " please", ".", "!"])).strip()


# -- say lines (the model's short spoken reply) --------------------------------------------

ACK = ["Tabii hocam.", "Hemen hocam.", "Olur hocam.", "Peki hocam.", "Tamam hocam.", "Hemen yapıyorum hocam.",
       "Emredersiniz hocam.", "Anlaşıldı hocam.", "Tabii hocam, hemen."]
TR_DIR_SAY = {"UP": "yukarı çıkıyorum", "DOWN": "aşağı iniyorum", "LEFT": "sola gidiyorum", "RIGHT": "sağa gidiyorum",
              "FORWARD": "ileri gidiyorum", "BACK": "geri çekiliyorum"}


def ack(specific=None):
    if specific and maybe(0.55):
        return specific
    return pick(ACK)


# -- worlds --------------------------------------------------------------------------------

POINT_NAMES = ["pick", "place", "kutu", "depo", "palet", "masa", "kamera", "istasyon", "bekleme", "ara", "test", "bant"]
SKILL_BANK = {
    "selam": ["MOVE_JOINT_REL J6 +30", "MOVE_JOINT_REL J6 -30"],
    "wave": ["MOVE_JOINT_REL J6 +30", "MOVE_JOINT_REL J6 -60", "MOVE_JOINT_REL J6 +30"],
    "nod": ["TILT_WRIST DOWN 10", "TILT_WRIST UP 10"],
    "pickup": ["MOVE_TCP DOWN 20", "GRIPPER CLOSE", "WAIT 0.5", "MOVE_TCP UP 20"],
    "salla": ["MOVE_JOINT_REL J1 +10", "MOVE_JOINT_REL J1 -10"],
    "tara": ["MOVE_TCP LEFT 50", "MOVE_TCP RIGHT 100", "MOVE_TCP LEFT 50"],
    "birak": ["MOVE_TCP DOWN 20", "GRIPPER OPEN", "MOVE_TCP UP 20"],
    "dans": ["MOVE_JOINT_REL J6 +45", "MOVE_JOINT_REL J6 -45"],
}


def make_world():
    names = []
    for i in range(1, 10):
        if maybe(0.35):
            names.append(f"P{i}")
    names += R.sample(POINT_NAMES, R.randint(0, 4))
    if not names and maybe(0.8):
        names = ["P1", "P2"]
    skills = {k: SKILL_BANK[k] for k in R.sample(sorted(SKILL_BANK), R.randint(0, 3))}
    return b.World(joints={"J1": -30.0, "J2": 20.0, "J3": 90.0, "J4": 0.0, "J5": 40.0, "J6": 0.0},
                   connected=maybe(0.95), servo_on=True, gripper="open",
                   points={n: {"joints": [-30, 10, 95, 0, 35, 0], "gripper": ""} for n in names},
                   home=[-30.0, 0.0, 100.0, 0.0, 30.0, 0.0], skills=skills)


# -- generators: each returns (request, say, commands) or None ---------------------------------

def g_tcp_tr(world):
    d = pick(list(TR_DIRS))
    words, intr, trans = TR_DIRS[d]
    amount, mm = tr_amount_mm()
    dw = pick(words)
    if amount == "MAX":
        amount = pick(["gidebildiğin kadar", "sonuna kadar", "sınıra kadar", "gidebildiğin yere kadar",
                       "maksimum"])
    form = R.random()
    if form < 0.5:
        text = f"{amount} {dw} {pick(intr)}"
    elif form < 0.72:
        text = f"{pick(TR_OBJECTS)} {amount} {dw} {pick(trans)}"
    elif form < 0.85:
        text = f"{dw} {amount} {pick(intr)}"
    else:
        text = f"{amount} {dw}" if amount else dw
    text = " ".join(text.split())
    cmd = f"MOVE_TCP {d} {mm:g}"
    say = ack(f"{mm_text(mm).capitalize()} {TR_DIR_SAY[d]} hocam." if mm != b.MAX_TCP_MM
              else "Gidebildiğim kadar gidiyorum hocam, sınırda dururum.")
    return with_frame(text), say, [cmd]


def g_tcp_en(world):
    d = pick(list(EN_DIRS))
    words, verbs, direct = EN_DIRS[d]
    amount, mm = en_amount_mm()
    if direct and maybe(0.3):
        text = f"{pick(direct)} {pick(['it', 'the arm', 'the gripper', ''])} {amount}"
    else:
        text = f"{pick(verbs)} {pick(words)} {amount}" if maybe(0.6) else f"{pick(verbs)} {amount} {pick(words)}"
    return with_frame(" ".join(text.split()), "en"), ack(), [f"MOVE_TCP {d} {mm:g}"]


JOINT_TR = {
    "J1": (["tabanı", "gövdeyi", "beli", "birinci eklemi", "1. eklemi", "J1'i", "taban eklemini"],
           ["döndür", "çevir", "döndürür müsün", "çevirir misin", "döndürelim"]),
    "J2": (["omzu", "omuz eklemini", "ikinci eklemi", "2. eklemi", "J2'yi"],
           ["hareket ettir", "oynat", "döndür", "çevir"]),
    "J3": (["dirseği", "dirsek eklemini", "üçüncü eklemi", "3. eklemi", "J3'ü"],
           ["hareket ettir", "oynat", "döndür", "bük"]),
    "J6": (["bileği", "tutucuyu", "bilek eklemini", "altıncı eklemi", "6. eklemi", "J6'yı", "gripperı"],
           ["döndür", "çevir", "döndürür müsün", "çevirir misin"]),
}


def g_joint_rel_tr(world):
    j = pick(list(JOINT_TR))
    names, verbs = JOINT_TR[j]
    amount, deg = tr_amount_deg()
    sign = +1
    mod = ""
    r = R.random()
    if j == "J1" and r < 0.35:
        side = pick(["sola", "sağa"])
        sign = +1 if side == "sola" else -1
        mod = side
    elif r < 0.55:
        mod = pick(["ters yönde", "eksi yönde", "geri", "diğer yöne"])
        sign = -1
    elif r < 0.65 and j in ("J2", "J3"):
        mod = pick(["ileri", "artı yönde"])
    if j in ("J2", "J3") and maybe(0.2):
        verbs = ["artır", "azalt"]
        v = pick(verbs)
        sign = +1 if v == "artır" else -1
        text = f"{pick(names).replace('yi', 'nin açısını').replace('ı', 'ın açısını', 1) if False else pick(names)} {amount} {v}"
    else:
        text = f"{pick(names)} {mod} {amount} {pick(verbs)}"
    text = " ".join(text.split())
    if amount and "derece" not in amount and maybe(0.5):
        text = text  # vague amounts stay vague
    cmd = f"MOVE_JOINT_REL {j} {sign * deg:+g}"
    return with_frame(text), ack(), [cmd]


def g_joint_rel_en(world):
    j, names = pick([("J1", ["the base", "the waist", "joint one", "joint 1", "J1"]),
                     ("J2", ["the shoulder", "joint two", "J2"]), ("J3", ["the elbow", "joint three", "J3"]),
                     ("J6", ["the wrist roll", "the gripper", "joint six", "J6"])])
    amount, deg = en_amount_deg()
    sign = -1 if maybe(0.3) else 1
    mod = pick(["the other way", "back", "minus"]) if sign < 0 else pick(["", "", "plus"])
    verb = pick(["rotate", "turn", "move", "spin"]) if j != "J6" else pick(["twist", "rotate", "roll", "turn"])
    text = f"{verb} {pick(names)} {mod} {amount}"
    return with_frame(" ".join(text.split()), "en"), ack(), [f"MOVE_JOINT_REL {j} {sign * deg:+g}"]


def g_tilt(world):
    d = pick(["UP", "DOWN"])
    if maybe(0.8):
        amount, deg = tr_amount_deg()
        obj = pick(["bileği", "bileği", "tutucuyu", "tutucunun ucunu", "bilek eklemini"])
        dw = "yukarı" if d == "UP" else "aşağı"
        verb = pick(["eğ", "eğer misin", "eğelim"] + (["kaldır", "bük"] if d == "UP" else ["indir", "bük"]))
        text = f"{obj} {amount} {dw} {verb}" if maybe(0.6) else f"{obj} {dw} {amount} {verb}"
        lang = "tr"
    else:
        amount, deg = en_amount_deg()
        text = f"tilt the wrist {d.lower()} {amount}" if maybe(0.6) else f"point the gripper {d.lower()}ward {amount}"
        lang = "en"
    return with_frame(" ".join(text.split()), lang), ack(), [f"TILT_WRIST {d} {deg:g}"]


def g_joint_abs(world):
    lims = b.DEFAULT_LIMITS
    k = 1 if maybe(0.75) else 2
    joints = R.sample(["J1", "J2", "J3", "J5", "J6"], k)
    names = {"J1": ["tabanı", "birinci eklemi", "J1'i"], "J2": ["omzu", "ikinci eklemi", "J2'yi"],
             "J3": ["dirseği", "üçüncü eklemi", "J3'ü"], "J5": ["bilek eğimini", "beşinci eklemi", "J5'i"],
             "J6": ["bilek dönüşünü", "altıncı eklemi", "J6'yı"]}
    items, parts = [], []
    for j in joints:
        lo, hi = lims[j]
        v = R.choice([0, 0, 10, 20, 30, 45, 60, 90, -10, -20, -30, -45, -90, 100, 120, -120])
        v = max(lo + 5, min(hi - 5, v))
        items.append(f"{j}={v:g}")
        parts.append(f"{pick(names[j])} {v:g} dereceye")
    verb = pick(["getir", "ayarla", "al", "getirir misin", "koy"])
    text = " ve ".join(parts) + " " + verb
    if joints == ["J6"] and items == ["J6=0"] and maybe(0.5):
        text = pick(["bileği düzelt", "bilek dönüşünü sıfırla", "tutucuyu düz konuma getir"])
    return with_frame(text), ack(), ["MOVE_JOINT_ABS " + " ".join(items)]


def g_gripper(world):
    close = maybe(0.5)
    if maybe(0.82):
        if close:
            text = pick(["tutucuyu kapat", "parçayı tut", "parçayı kavra", "kıskacı kapat", "gripperı kapat",
                         "tut", "parçayı sıkıca tut", "yakala", "parçayı yakala", "tutucu kapansın", "pensi kapat",
                         "sık", "tutucuyu kapatır mısın"])
        else:
            text = pick(["tutucuyu aç", "parçayı bırak", "bırak", "serbest bırak", "kıskacı aç", "gripperı aç",
                         "tutucu açılsın", "pensi aç", "parçayı serbest bırak", "tutucuyu açar mısın", "aç tutucuyu"])
        text, lang = with_frame(text), "tr"
    else:
        text = pick(["close the gripper", "grab it", "grip the part", "hold it"] if close else
                    ["open the gripper", "release it", "let go", "drop the part"])
        text, lang = with_frame(text, "en"), "en"
    return text, ack("Tutucuyu kapatıyorum hocam." if close else "Tutucuyu açıyorum hocam."), \
        ["GRIPPER CLOSE" if close else "GRIPPER OPEN"]


def g_speed(world):
    r = R.random()
    if r < 0.55:
        n = pick([1, 2, 3, 5, 5, 8, 10, 10, 12, 15, 15, 20, 20, 25, 30])
        text = pick([f"hızı yüzde {say_num(n)} yap", f"hız yüzde {say_num(n)} olsun", f"yüzde {say_num(n)} hıza geç",
                     f"hızı %{n} yap", f"yüzde {say_num(n)} hızla çalış", f"hızı {say_num(n)} yap",
                     f"yüzde {say_num(n)} hızla devam et"])
        return with_frame(text), ack(f"Hızı yüzde {n} yapıyorum hocam."), [f"SPEED {n}"]
    if r < 0.7:
        n = pick([40, 50, 60, 80, 100])
        text = pick([f"hızı yüzde {say_num(n)} yap", f"yüzde {n} hızla git"])
        return with_frame(text), f"Yapay zekâ ile en fazla yüzde 30'a izin var hocam, 30 yapıyorum.", ["SPEED 30"]
    if r < 0.8:
        text = pick(["tam hız", "maksimum hıza çık", "en hızlı moda geç", "hızı maksimuma al", "tam hızla çalış"])
        return with_frame(text), "İzin verilen en yüksek hız yüzde 30, onu ayarlıyorum hocam.", ["SPEED 30"]
    if r < 0.9:
        text = pick(["yavaşla", "biraz yavaşla", "yavaş çalış", "daha yavaş git", "hızı düşür"])
        return with_frame(text), ack("Yavaşlıyorum hocam, yüzde 10."), ["SPEED 10"]
    n = pick([5, 10, 15, 20, 25])
    return with_frame(pick([f"set speed to {n} percent", f"run at {n} percent", f"speed {n}%"]), "en"), ack(), \
        [f"SPEED {n}"]


def g_wait(world):
    s = pick([0.5, 1, 1, 2, 2, 3, 5, 5, 10])
    spoken = "yarım" if s == 0.5 else say_num(s)
    text = pick([f"{spoken} saniye bekle", f"{spoken} saniye bekleyelim", f"{spoken} saniye beklesin",
                 f"{spoken} saniye bekle sonra devam ederiz"])
    if maybe(0.1):
        s2 = pick([20, 30, 60])
        return with_frame(f"{s2} saniye bekle"), "En fazla 10 saniye bekleyebilirim hocam, 10 saniye bekliyorum.", \
            ["WAIT 10"]
    return with_frame(text), ack(), [f"WAIT {s:g}"]


def g_home(world):
    if maybe(0.85):
        text = pick(["eve dön", "eve git", "başlangıç konumuna dön", "başlangıç pozisyonuna git", "home'a git",
                     "ev konumuna git", "başa dön", "ev pozisyonuna dön", "başlangıca dön", "evine dön",
                     "home pozisyonuna git", "başlangıç noktasına git"])
        return with_frame(text), ack("Başlangıç konumuna gidiyorum hocam."), ["GO_HOME"]
    return with_frame(pick(["go home", "back to the start position", "home position please"]), "en"), ack(), \
        ["GO_HOME"]


def point_phrase(name):
    m = re.fullmatch(r"P(\d)", name)
    opts = [f"{dative(name)} git", f"{name} noktasına git", f"{name} konumuna git", f"{dative(name)} gider misin",
            f"{name} noktasına gidelim"]
    if m:
        n = int(m[1])
        ordinal = ["", "birinci", "ikinci", "üçüncü", "dördüncü", "beşinci", "altıncı", "yedinci", "sekizinci",
                   "dokuzuncu"][n]
        opts += [f"{n} numaralı noktaya git", f"{ordinal} noktaya git", f"{tr_num(n)} numaralı noktaya git",
                 f"P {n} noktasına git", f"pe {tr_num(n)} noktasına git"]
    return pick(opts)


def g_goto(world):
    names = list(world.points)
    if not names:
        return None
    if maybe(0.12):
        # a point that does not exist: no motion, say which ones exist
        missing = pick([f"P{i}" for i in range(1, 10) if f"P{i}" not in names] + [n for n in POINT_NAMES if n not in names])
        say = f"{missing} diye kayıtlı bir nokta yok hocam. Kayıtlı noktalar: {', '.join(names)}."
        return with_frame(point_phrase(missing)), say, []
    if len(names) >= 2 and maybe(0.25):
        a, c = R.sample(names, 2)
        text = f"{dative(a)} git{pick([', sonra ', ' ve ardından ', ' sonra '])}{dative(c)} git"
        return with_frame(text), ack(), [f"GOTO_POINT {a}", f"GOTO_POINT {c}"]
    n = pick(names)
    if maybe(0.12):
        return with_frame(pick([f"go to point {n}", f"move to {n}", f"head to {n}"]), "en"), ack(), [f"GOTO_POINT {n}"]
    return with_frame(point_phrase(n)), ack(f"{n} noktasına gidiyorum hocam."), [f"GOTO_POINT {n}"]


NEW_NAMES = ["kutu", "masa", "alma", "birakma", "ara", "depo", "palet", "bekleme", "kamera", "orta", "test",
             "nokta1", "hedef", "baslangic", "sol", "sag"]


def g_save(world):
    n = pick(NEW_NAMES + [f"P{i}" for i in range(1, 10)])
    text = pick([f"bu konumu {n} olarak kaydet", f"burayı {n} diye kaydet", f"şu anki pozisyonu {n} adıyla kaydet",
                 f"bu noktayı {n} olarak hatırla", f"burayı {n} olarak kaydeder misin", f"bu pozisyonu {n} diye sakla",
                 f"bu konumu {n} adıyla kaydet"])
    if maybe(0.12):
        text = pick([f"save this position as {n}", f"remember this pose as {n}"])
        return with_frame(text, "en"), ack(), [f"SAVE_POINT {n}"]
    return with_frame(text), ack(f"Bu konumu {n} olarak kaydediyorum hocam."), [f"SAVE_POINT {n}"]


def g_shape(world):
    kind = pick(["square", "square", "circle", "circle", "rect", "arc", "arc"])
    vertical = maybe(0.3)
    vword = pick(["dikey ", "düşey ", "dik duran ", "dikey olarak "]) if vertical else pick(["", "", "yatay ", ""])
    verb = pick(["çiz", "çizer misin", "çizebilir misin", "çizelim", "yap", "çiz bakalım"])
    suffix = " VERTICAL" if vertical else ""
    if kind == "square":
        r = R.random()
        if r < 0.55:
            cm = pick([2, 3, 4, 5, 6, 8, 10, 12, 15])
            size = pick([f"{say_num(cm)} santimlik ", f"kenarı {say_num(cm)} santim olan ", f"{cm} cm kenarlı ",
                         f"{say_num(cm)} santim kenarlı "])
            mm = cm * 10
        elif r < 0.7:
            mm = pick([30, 40, 50, 60, 80])
            size = f"{mm} milimetrelik "
        elif r < 0.85:
            size, mm = pick([("küçük ", 50), ("büyük ", 100)])
        else:
            size, mm = "", 50
        text = f"{size}{vword}bir kare {verb}" if maybe(0.6) else f"{vword}{size}kare {verb}"
        return with_frame(" ".join(text.split())), ack(f"{mm / 10:g} santim kenarlı kare çiziyorum hocam.".replace(".", ",", 1) if False else None), \
            [f"DRAW_SQUARE {mm:g}{suffix}"]
    if kind == "circle":
        noun = pick(["daire", "daire", "çember", "yuvarlak", "halka"])
        r = R.random()
        if r < 0.45:
            cm = pick([2, 3, 4, 5, 6, 8, 10, 12])
            size = pick([f"{say_num(cm)} santimlik ", f"çapı {say_num(cm)} santim olan ", f"{cm} cm çaplı "])
            mm = cm * 10
        elif r < 0.65:
            cm = pick([2, 3, 4, 5])
            size = pick([f"yarıçapı {say_num(cm)} santim olan ", f"{say_num(cm)} santim yarıçaplı "])
            mm = cm * 20
        elif r < 0.8:
            size, mm = pick([("küçük ", 50), ("büyük ", 100)])
        else:
            size, mm = "", 50
        text = f"{size}{vword}bir {noun} {verb}"
        return with_frame(" ".join(text.split())), ack(), [f"DRAW_CIRCLE {mm:g}{suffix}"]
    if kind == "rect":
        w, h = R.sample([2, 3, 4, 5, 6, 8, 10, 12], 2)
        size = pick([f"{w} santime {h} santim ", f"{w}'e {h} santimlik ".replace("'e", "'" + ("a" if last_vowel(tr_num(w)) == "back" else "e")),
                     f"eni {w} boyu {h} santim olan ", f"{w} cm'ye {h} cm "])
        text = f"{size}{vword}bir dikdörtgen {verb}"
        return with_frame(" ".join(text.split())), ack(), [f"DRAW_RECTANGLE {w * 10} {h * 10}{suffix}"]
    # arc
    r = R.random()
    if r < 0.4:
        deg, name = 180, pick(["yarım daire", "yarım çember", "yarım bir yay"])
    elif r < 0.7:
        deg, name = 90, pick(["çeyrek daire", "çeyrek çember", "çeyrek yay"])
    else:
        deg = pick([45, 60, 120, 135, 270])
        name = f"{say_num(deg)} derecelik bir yay"
    rad = ""
    radius = 50
    if maybe(0.5):
        cm = pick([2, 3, 4, 5, 6, 8])
        rad = pick([f"yarıçapı {say_num(cm)} santim olan ", f"{say_num(cm)} santim yarıçaplı "])
        radius = cm * 10
    side = "LEFT"
    sw = ""
    if maybe(0.6):
        side = pick(["LEFT", "RIGHT"])
        sw = pick(["sola doğru ", "sola kıvrılan "]) if side == "LEFT" else pick(["sağa doğru ", "sağa kıvrılan "])
    text = f"{sw}{rad}{vword}{name} {verb}"
    return with_frame(" ".join(text.split())), ack(), [f"DRAW_ARC {radius} {deg} {side}{suffix}"]


def g_shape_en(world):
    kind = pick(["square", "circle", "arc"])
    cm = pick([3, 4, 5, 6, 8, 10])
    vertical = maybe(0.3)
    suffix = " VERTICAL" if vertical else ""
    v = "vertical " if vertical else ""
    if kind == "square":
        return with_frame(f"draw a {cm} cm {v}square", "en"), ack(), [f"DRAW_SQUARE {cm * 10}{suffix}"]
    if kind == "circle":
        return with_frame(f"draw a {v}circle {cm} cm across", "en"), ack(), [f"DRAW_CIRCLE {cm * 10}{suffix}"]
    side = pick(["LEFT", "RIGHT"])
    return with_frame(f"make a {v}half circle to the {side.lower()}", "en"), ack(), [f"DRAW_ARC 50 180 {side}{suffix}"]


def g_line(world):
    r = R.random()
    if r < 0.35:
        text = pick(["sipariş var mı", "istasyondan sipariş geldi mi", "hat ne durumda", "transfer hattını kontrol et",
                     "istasyon ne istiyor", "yeni iş var mı", "4. istasyonun durumu ne", "sipariş durumu nedir",
                     "hattan bir şey geldi mi", "istasyondan ne istiyorlar", "hangi sipariş var",
                     "transfer hattı ne diyor", "is there an order from the line", "check the station"])
        return with_frame(text), pick(["Hemen bakıyorum hocam.", "Hattı kontrol ediyorum hocam.", "Bakıyorum hocam."]), \
            ["LINE_STATUS"]
    if r < 0.5:
        text = pick(["istasyona hazır olduğumuzu söyle", "hazır sinyalini ver", "hazır sinyalini aç", "hazırız de",
                     "istasyona hazırız diye bildir", "hazır sinyali gönder", "tell the station we are ready"])
        return with_frame(text), ack(), ["LINE_READY ON"]
    if r < 0.58:
        text = pick(["hazır sinyalini kapat", "hazır değiliz de", "hazır sinyalini kes", "istasyona hazır olmadığımızı söyle"])
        return with_frame(text), ack(), ["LINE_READY OFF"]
    if r < 0.7:
        text = pick(["meşgul sinyalini aç", "siparişe başladık, meşgul yap", "çalıştığımızı bildir", "meşgul yap",
                     "istasyona meşgul olduğumuzu söyle", "işe başladık, meşgul sinyali ver"])
        return with_frame(text), ack(), ["LINE_BUSY ON"]
    if r < 0.8:
        text = pick(["meşgulü kapat", "meşgul sinyalini kapat", "işimiz bitti, meşgulü kaldır", "meşgul sinyalini kes"])
        return with_frame(text), ack(), ["LINE_BUSY OFF"]
    if r < 0.87:
        text = pick(["iş bitti, meşgulü kapat ve hazır ver", "sipariş tamamlandı, meşgulü kapatıp hazır sinyali ver"])
        return with_frame(text), ack(), ["LINE_BUSY OFF", "LINE_READY ON"]
    if r < 0.95:
        n = pick([1, 2, 3])
        text = pick([f"istasyona hata kodu {n} gönder", f"{n} numaralı hatayı bildir", f"hata kodu {say_num(n)} ver",
                     f"istasyona {n} hata kodunu gönder"])
        return with_frame(text), ack(), [f"LINE_ERROR_CODE {n}"]
    return with_frame(pick(["hatayı temizle", "hata kodunu sıfırla", "hata kodunu kaldır", "hatayı sil"])), ack(), \
        ["LINE_ERROR_CODE 0"]


def g_read(world):
    r = R.random()
    if r < 0.4:
        text = pick(["neredesin", "şu an neredesin", "konumun ne", "eklem açıları kaç", "pozisyonu söyle",
                     "koordinatların ne", "tutucu nerede", "açılar ne durumda", "şimdi neredesin acaba",
                     "konumunu söyler misin", "hangi pozisyondasın", "eklemler kaç derecede", "where are you",
                     "what are the joint angles"])
        return with_frame(text, question=True), pick(["Hemen bakıyorum hocam.", "Bakıyorum hocam.", "Hemen söylüyorum hocam."]), ["WHERE"]
    if r < 0.7:
        text = pick(["servo açık mı", "alarm var mı", "durum nasıl", "bağlı mısın", "tutucu açık mı kapalı mı",
                     "robotta alarm var mı", "servo durumu ne", "her şey yolunda mı", "robotun durumu nedir",
                     "tutucu açık mı", "bağlantı var mı", "is the servo on", "any alarms"])
        return with_frame(text, question=True), pick(["Hemen bakıyorum hocam.", "Kontrol ediyorum hocam."]), ["STATUS"]
    if r < 0.85:
        text = pick(["hangi noktaları biliyorsun", "kayıtlı noktalar neler", "noktaları listele",
                     "hangi konumlar kayıtlı", "kaydettiğin noktalar ne"])
        return with_frame(text), ack(), ["LIST_POINTS"]
    text = pick(["hangi becerileri biliyorsun", "becerilerini say", "neler öğrendin", "öğrettiğim beceriler neler",
                 "beceri listesini göster"])
    return with_frame(text), ack(), ["LIST_SKILLS"]


def tr_step_phrase():
    """A short step for skills and sequences: (text, [commands])."""
    r = R.random()
    if r < 0.55:
        d = pick(list(TR_DIRS))
        cm = pick([1, 2, 3, 4, 5, 10])
        verb = {"UP": "çık", "DOWN": "in", "LEFT": "git", "RIGHT": "git", "FORWARD": "git", "BACK": "gel"}[d]
        return f"{say_num(cm)} santim {TR_DIRS[d][0][0]} {verb}", [f"MOVE_TCP {d} {cm * 10}"]
    if r < 0.7:
        return pick([("tutucuyu kapat", ["GRIPPER CLOSE"]), ("parçayı tut", ["GRIPPER CLOSE"]),
                     ("tutucuyu aç", ["GRIPPER OPEN"]), ("parçayı bırak", ["GRIPPER OPEN"])])
    if r < 0.8:
        d = pick([15, 30, 45])
        return pick([(f"bileği {d} derece döndür", [f"MOVE_JOINT_REL J6 +{d}"]),
                     (f"tabanı {d} derece döndür", [f"MOVE_JOINT_REL J1 +{d}"])])
    if r < 0.88:
        s = pick([1, 2, 3])
        return f"{say_num(s)} saniye bekle", [f"WAIT {s}"]
    return "eve dön", ["GO_HOME"]


def add_grip_pause(cmds):
    """A short settling pause after closing the gripper when more follows (as the seed examples do)."""
    out = []
    for i, c in enumerate(cmds):
        out.append(c)
        if c == "GRIPPER CLOSE" and i + 1 < len(cmds) and not cmds[i + 1].startswith("WAIT"):
            out.append("WAIT 0.5")
    return out


def g_sequence(world):
    n = R.randint(2, 4)
    parts, cmds = [], []
    for _ in range(n):
        t, c = tr_step_phrase()
        if t in parts:
            continue
        parts.append(t)
        cmds += c
    if len(parts) < 2:
        return None
    if R.random() < 0.3:  # the classic pick
        cm = pick([2, 3, 4, 5])
        parts = [f"{say_num(cm)} santim in", pick(["parçayı tut", "parçayı kavra", "tutucuyu kapat"]),
                 f"{say_num(cm)} santim {pick(['kalk', 'yukarı çık'])}"]
        cmds = [f"MOVE_TCP DOWN {cm * 10}", "GRIPPER CLOSE", f"MOVE_TCP UP {cm * 10}"]
    joiners = [", sonra ", " ve ", ", ardından ", " sonra da ", ", "]
    text = parts[0]
    for p in parts[1:]:
        text += pick(joiners) + p
    return with_frame(text), ack(), add_grip_pause(cmds)


def g_skill(world):
    names = list(world.skills)
    r = R.random()
    if r < 0.4 and names:
        n = pick(names)
        if maybe(0.15):
            return with_frame(pick([f"iki kere {n} yap", f"{n} becerisini iki defa yap"])), ack(), [f"RUN_SKILL {n}"] * 2
        text = pick([f"{n} becerisini yap", f"{n} becerisini çalıştır", f"{n} yap", f"{n} hareketini yap",
                     f"{n} becerisini uygula", f"do the {n} skill"])
        return with_frame(text), ack(f"{n} becerisini yapıyorum hocam."), [f"RUN_SKILL {n}"]
    if r < 0.5:
        missing = pick([k for k in SKILL_BANK if k not in names] or ["zıpla"])
        return with_frame(f"{missing} becerisini yap"), \
            f"{missing} diye bir beceri bilmiyorum hocam. İsterseniz adımlarını söyleyin, öğreneyim.", []
    if r < 0.85:
        n = pick(["selam", "salla", "tara", "kontrol", "dans", "pickup", "nod", "birak", "al", "test", "yokla"])
        k = R.randint(2, 3)
        parts, cmds = [], []
        for _ in range(k):
            t, c = tr_step_phrase()
            if c == ["GO_HOME"]:
                continue
            parts.append(t)
            cmds += c
        if len(cmds) < 2:
            return None
        cmds = add_grip_pause(cmds)
        body = pick([", sonra ", " ve ", ", "]).join(parts)
        text = pick([f"{n} adında bir beceri öğren: {body}", f"ben {n} deyince {body}",
                     f"{n} diye bir beceri öğret: {body}", f"yeni beceri {n}: {body}"])
        return with_frame(text), f"{n} becerisini öğreniyorum hocam.", [f"DEFINE_SKILL {n}: " + " | ".join(cmds)]
    if names:
        n = pick(names)
        return with_frame(pick([f"{n} becerisini unut", f"{n} becerisini sil", f"{n} becerisini kaldır"])), ack(), \
            [f"FORGET_SKILL {n}"]
    return None


def g_servo_conveyor(world):
    r = R.random()
    if r < 0.3:
        return with_frame(pick(["servoyu kapat", "motorları kapat", "servo off", "servoyu kes", "servoları kapatır mısın",
                                "turn the servo off"])), ack("Servoyu kapatıyorum hocam."), ["SERVO_OFF"]
    if r < 0.65:
        return with_frame(pick(["servoyu aç", "motorları aç", "motorları çalıştır", "servo on", "servoyu açar mısın",
                                "servoları aktif et", "turn the servo on", "enable the servo"])), \
            pick(["Servoyu ben açamam hocam; lütfen arayüzdeki Servo ON düğmesine basın.",
                  "Güvenlik gereği servoyu açmak sizde hocam; Servo ON düğmesini kullanın lütfen."]), []
    if r < 0.85:
        return with_frame(pick(["bandı çalıştır", "konveyörü başlat", "bant hareket etsin", "konveyörü çalıştır",
                                "bandı ilerlet", "start the conveyor"])), \
            pick(["Konveyörü robot kontrol etmiyor hocam; bant istasyonun kendi kontrolünde.",
                  "Bandı ben çalıştıramam hocam, o transfer hattının kontrolünde."]), []
    return with_frame(pick(["4. eklemi döndür", "dördüncü eklemi 10 derece çevir", "J4'ü hareket ettir", "move joint 4"])), \
        "Bu robotta 4. eklem yok hocam; eklemler J1, J2, J3, J5 ve J6.", []


CHAT = [
    (["merhaba", "selam", "merhaba hocam", "selamlar", "günaydın", "iyi akşamlar", "hey robot", "hello"],
     ["Merhaba hocam! Bugün kolla ne yapalım?", "Merhaba hocam, emrinizdeyim.", "Selam hocam! Hazırım, ne yapıyoruz?",
      "Günaydın hocam, sistemler hazır."]),
    (["nasılsın", "naber", "nasıl gidiyor", "iyi misin", "how are you"],
     ["Çok iyiyim hocam, teşekkürler. Siz nasılsınız?", "Gayet iyiyim hocam, çalışmaya hazırım.",
      "İyiyim hocam, eklemler yağlı, motorlar sıcak!"]),
    (["teşekkürler", "sağ ol", "eline sağlık", "harikaydı", "güzel oldu", "aferin", "thanks", "süper"],
     ["Rica ederim hocam! Başka bir şey yapalım mı?", "Ne demek hocam, her zaman.", "Teşekkürler hocam, devam edelim mi?"]),
    (["ne yapabilirsin", "neler yapabiliyorsun", "yeteneklerin neler", "what can you do"],
     ["Aracı her yöne hareket ettirebilir, eklemleri döndürebilir, tutucuyu açıp kapatabilir, kare, daire ve yay "
      "çizebilir, noktaları kaydedip gidebilir ve yeni beceriler öğrenebilirim hocam."]),
    (["kaç eksenin var", "bu robotun kaç ekseni var", "kaç eklemin var"],
     ["Beş eksenim var hocam: taban, omuz, dirsek, bilek eğimi ve bilek dönüşü. Dördüncü eklem yok."]),
    (["tutucu ne kadar güçlü", "gripper ne kadar sıkar", "tutucun ne"],
     ["Festo HGP-16 pnömatik paralel tutucu, 160 newtona kadar sıkabilir hocam."]),
    (["sen kimsin", "adın ne", "kimsin sen"],
     ["Ben RV-2AJ robot kolunun asistanıyım hocam. Söyleyin, yapayım.", "RV-2AJ'nin yapay zekâ asistanıyım hocam."]),
    (["bana bir şaka yap", "şaka yap", "beni güldür"],
     ["Robotlar neden hiç yorulmaz hocam? Çünkü her zaman şarjları dolu!",
      "Bir robot kafeye girmiş, garson sormuş: ne alırsınız? Robot: biraz yağ, sade lütfen hocam."]),
    (["yoruldum", "biraz dinlenelim", "mola verelim", "ara verelim"],
     ["Tabii hocam, dinlenelim. Hazır olunca söylemeniz yeterli.", "Olur hocam, ben burada bekliyorum."]),
    (["görüşürüz", "hoşça kal", "bugünlük bu kadar", "iyi geceler"],
     ["Görüşürüz hocam, iyi çalışmalar!", "Hoşça kalın hocam, kol güvenli konumda bekliyor."]),
    (["sence ne yapalım", "ne yapalım", "bir öneri ver"],
     ["İsterseniz küçük bir kare çizebilirim ya da bir parçayı alıp başka bir noktaya bırakabiliriz hocam."]),
    (["hızın ne kadar olabilir", "en fazla ne kadar hızlı gidebilirsin"],
     ["Yapay zekâ ile en fazla yüzde 30 hıza izin veriyorum hocam, güvenlik için."]),
    (["bu kol ne işe yarıyor", "bu robot ne yapar"],
     ["Parçaları alıp taşır, montaj yapar ve Festo transfer hattının 4. istasyonuyla birlikte çalışır hocam."]),
    (["hareket et", "bir şey yap", "oynat"],
     ["Hangi yöne ve ne kadar gideyim hocam? Örneğin “2 santim sola git”.",
      "Nereye gideyim hocam? Yön ve mesafe söylerseniz hemen yaparım."]),
]


def g_chat(world):
    reqs, says = pick(CHAT)
    return with_frame(pick(reqs), question=True), pick(says), []


def g_mixed(world):
    """Small talk plus a command in one sentence."""
    lead_req, lead_say = pick([("günaydın hocam", "Günaydın hocam!"), ("merhaba", "Merhaba hocam!"),
                               ("tamam güzel", "Sevindim hocam!"), ("harika oldu", "Teşekkürler hocam!"),
                               ("bugün biraz yorgunum", "Geçmiş olsun hocam, dikkatli gidelim."),
                               ("şimdi bir deneme yapalım", "Tabii hocam."), ("hadi başlayalım", "Hadi hocam!")])
    gen = pick([g_tcp_tr, g_gripper, g_shape, g_home, g_goto, g_speed])
    out = gen(world)
    if out is None or not out[2]:
        return None
    req, _say, cmds = out
    return f"{lead_req}, {req[0].lower() + req[1:]}", lead_say, cmds


# -- speech-recognition noise ------------------------------------------------------------------

_ASCII = str.maketrans({"ı": "i", "ş": "s", "ğ": "g", "ü": "u", "ö": "o", "ç": "c", "İ": "I"})


def noisy(text):
    r = R.random()
    if r < 0.35:
        return text.translate(_ASCII)
    if r < 0.55:
        return text[0].upper() + text[1:] + pick([".", ".", "?", ""])
    if r < 0.7:
        return pick(["ııı ", "şey, ", "eee ", "hmm "]) + text
    if r < 0.85:
        words = text.split()
        i = R.randrange(len(words))
        w = words[i]
        if len(w) > 3:
            j = R.randrange(1, len(w) - 1)
            w = w[:j] + w[j] + w[j:]  # a doubled letter ("illeri")
        words[i] = w
        return " ".join(words)
    return text.replace("yukarı", "yukarıya").replace("aşağı", "aşşağı").replace("santim", "santim")


# -- follow-ups (with the previous turn as history) ---------------------------------------------

def g_followup(world):
    base = pick([g_tcp_tr, g_tcp_tr, g_joint_rel_tr, g_tilt, g_shape, g_gripper])(world)
    if base is None or not base[2]:
        return None
    prev_req, prev_say, prev_cmds = base
    previous = {"request": prev_req, "commands": prev_cmds}
    kind = R.random()
    if kind < 0.3:
        req = pick(["biraz daha", "daha", "biraz daha lütfen", "bir tık daha", "azıcık daha", "devam", "daha fazla",
                    "biraz daha hocam", "a bit more"])
    elif kind < 0.45:
        cm = pick([1, 2, 3, 5])
        req = pick([f"{say_num(cm)} santim daha", f"{cm} cm daha", f"bir {say_num(cm)} santim daha"])
    elif kind < 0.62:
        req = pick(["tekrar", "bir daha", "tekrarla", "aynısını yap", "bir kez daha", "tekrar et", "again"])
    elif kind < 0.78:
        req = pick(["geri al", "geri al lütfen", "onu geri al", "yaptığını geri al", "undo"])
    elif kind < 0.9 and match_name(prev_cmds[0]) in b.SHAPES:
        req = pick(["biraz büyüt", "daha büyük yap", "biraz küçült", "2 santim büyüt", "daha küçük olsun",
                    "biraz daha büyütebilir misin", "bir santim küçült"])
    else:
        d = pick(list(TR_DIRS))
        cm = pick([1, 2, 3, 5])
        req = pick([f"şimdi de {say_num(cm)} santim {TR_DIRS[d][0][0]}", f"şimdi {TR_DIRS[d][0][0]} git"])
        mm = cm * 10 if "santim" in req else 20
        return (prev_req, prev_say, prev_cmds), req, ack(), [f"MOVE_TCP {d} {mm}"]
    cmds = b.resolve_followup(req, previous)
    if not cmds:
        return None
    cmds = [arc_words(c) for c in cmds]
    return (prev_req, prev_say, prev_cmds), req, ack(), cmds


def arc_words(cmd):
    """DRAW_ARC 50 -90 -> DRAW_ARC 50 90 RIGHT (the form the model writes)."""
    m = re.fullmatch(r"DRAW_ARC (\S+) ([+-]?[\d.]+)( VERTICAL)?", cmd)
    if not m:
        return cmd
    deg = float(m[2])
    return f"DRAW_ARC {m[1]} {abs(deg):g} {'RIGHT' if deg < 0 else 'LEFT'}{m[3] or ''}"


def match_name(cmd):
    return b.match(b.normalize(cmd))[0]


# -- knowledge, concepts, explanations, troubleshooting -------------------------------------------

KNOWLEDGE = [
    (["bu robotun modeli ne", "hangi robotsun", "sen hangi kolsun", "robotun markası ne", "what robot is this"],
     ["Ben Mitsubishi Electric MELFA RV-2AJ hocam; 5 eksenli, dikey mafsallı küçük bir endüstriyel kolum."]),
    (["kaç eksenin var", "kaç eklemin var", "eklemlerin neler", "hangi eklemler var", "eksenlerini say"],
     ["Beş eksenim var hocam: J1 taban, J2 omuz, J3 dirsek, J5 bilek eğimi ve J6 bilek dönüşü. J4 yok."]),
    (["J4 neden yok", "dördüncü eklem nerede", "neden 6 eksenli değilsin", "4. eksen niye yok"],
     ["RV-2AJ 5 eksenli bir model hocam, J4 hiç yok. Bu yüzden tutucu eğilip dönebiliyor ama yana yatamıyor; "
      "bazı yönelimlere 6 eksenli bir kol kadar serbest ulaşamam."]),
    (["ne kadar yük kaldırabilirsin", "taşıma kapasiten ne", "kaç kilo kaldırırsın", "maksimum yükün ne"],
     ["Katalog değerine göre en fazla 2 kilogram taşıyabilirim hocam; tutucunun ağırlığı da buna dahil."]),
    (["erişim mesafen ne", "ne kadar uzağa uzanabilirsin", "kolun ne kadar uzun", "menzilin ne"],
     ["Erişim yarıçapım yaklaşık 410 milimetre hocam. Üst kolum 250, ön kolum 160 milimetre; bilekten flanşa 72 milimetre daha var."]),
    (["ne kadar hassassın", "tekrarlanabilirliğin ne", "hassasiyetin ne kadar", "aynı noktaya ne kadar doğru dönersin"],
     ["Tekrarlanabilirliğim artı eksi 0,02 milimetre hocam; yani aynı noktaya tekrar tekrar çok hassas dönebilirim."]),
    (["kaç kilosun", "ağırlığın ne", "robot ne kadar ağır"], ["Yaklaşık 17 kilogramım hocam."]),
    (["omuz yüksekliği ne", "omuz ekseni kaç milimetrede", "tabandan omza mesafe ne"],
     ["Omuz eklemim (J2) montaj yüzeyinden 300 milimetre yukarıda hocam."]),
    (["kontrolörün ne", "hangi kontrolörü kullanıyorsun", "kontrol ünitesi hangisi"],
     ["CR1-571 kontrolörüyle çalışıyorum hocam; programlama dili MELFA-BASIC IV."]),
    (["bilgisayarla nasıl haberleşiyorsun", "bağlantı nasıl", "hangi kabloyla bağlısın", "iletişim protokolü ne"],
     ["RS-232 seri kabloyla, USB-seri adaptör üzerinden hocam: 9600 baud, 8 veri biti, çift parite, 2 stop biti. "
      "Komutlar 1;1; ile başlar, Qo cevabı tamam, Qe veya Qx hata demek."]),
    (["tutucun ne", "gripper hangi model", "tutucu ne kadar güçlü", "kaç newton sıkıyorsun", "tutucu nasıl çalışıyor"],
     ["Festo HGP-16 pnömatik paralel tutucum var hocam; hava ile açılıp kapanıyor ve 160 newtona kadar sıkabiliyor."]),
    (["parçayı tuttuktan sonra neden bekliyorsun", "kapattıktan sonra neden yarım saniye bekliyorsun"],
     ["Tutucu havayla çalıştığı için kapanıp parçayı oturtması yarım saniye kadar sürüyor hocam; beklemeden kalkarsam parça kayabilir."]),
    (["istasyonla nasıl konuşuyorsun", "transfer hattıyla bağlantın ne", "Festo ile nasıl haberleşiyorsun"],
     ["Festo MPS 500 hattının 4. istasyonuna XPA4 kablosuyla bağlıyım hocam. Siparişi IN6 ve IN7 girişlerinden okuyorum; "
      "hazır, meşgul ve hata kodu sinyallerini OUT4-OUT7 çıkışlarıyla gönderiyorum."]),
    (["sipariş kodları ne demek", "hangi siparişler var", "sipariş türleri neler"],
     ["İki bitlik kod hocam: 0 sipariş yok, 1 montaj, 2 ayıklama, 3 paletten parça al. İsterseniz şu anki siparişe bakayım."]),
    (["bandı neden çalıştıramıyorsun", "konveyörü neden kontrol edemiyorsun", "bant kimde"],
     ["Konveyörü hattın kendi PLC'si yönetiyor hocam; bana bağlı bir bant sinyali yok. Ben sadece hazır, meşgul ve hata kodu gönderebiliyorum."]),
    (["OUT0 ne işe yarıyor", "ilk dört çıkış ne", "OUT3'ü açar mısın"],
     ["OUT0-OUT3 bu hücrede bağlı değil hocam, o yüzden onlara dokunmuyorum. Kullanılan çıkışlar OUT4-OUT7."]),
    (["nasıl hareket planlıyorsun", "hareketleri nasıl hesaplıyorsun", "MoveIt ne yapıyor burada"],
     ["Her hareketi ROS 2 üzerinde MoveIt ile planlıyorum hocam; yol çarpışma kontrolünden geçiyor, düz hareketler 50 milimetrelik parçalara bölünüyor."]),
    (["komutlarımı nasıl kontrol ediyorsun", "yanlış bir şey yaparsan ne olur", "güvenli misin"],
     ["Söylediğiniz her şey önce doğrulayıcıdan geçiyor hocam: eklem sınırları, hız, yasak çıkışlar. Sonra planı size gösterip onayınızı bekliyorum; "
      "ayrıca 'dur' demeniz beni anında durdurur."]),
    (["servoyu neden açamıyorsun", "servoyu sen aç", "neden servoyu ben açıyorum"],
     ["Güvenlik kuralı hocam: servoyu yapay zekâ asla açmaz, robotun başında bir insan olduğundan emin olmak için Servo ON düğmesine siz basıyorsunuz. Kapatmayı ise yapabilirim."]),
    (["seni nasıl durdururum", "acil durumda ne yapmalıyım", "durdurmak için ne demeliyim"],
     ["'Dur' ya da 'stop' demeniz yeterli hocam, dil modelini beklemeden hemen dururum. Ekrandaki STOP düğmesi ve donanım acil stop butonu da her zaman çalışır; en güçlüsü acil stop."]),
    (["hız sınırın ne", "neden yüzde 30'dan hızlı gitmiyorsun", "maksimum hızın ne"],
     ["Yapay zekâ ile en fazla yüzde 30 hıza izin veriyorum hocam; sesle komut verirken hata payını küçük tutmak için."]),
    (["eklem sınırların ne", "hangi açılara gidebilirsin", "limitlerin ne"],
     ["Yapay zekâ sınırlarım: J1 -150 ile 150, J2 -60 ile 120, J3 -110 ile 155, J5 -90 ile 90, J6 -200 ile 200 derece hocam. Kontrolörün kendi sınırları da ayrıca geçerli."]),
    (["J3 neden 155'e kadar", "dirsek sınırı neden farklı", "J3 kalibrasyonu ne"],
     ["Bu robotta J3'ün orijini biraz kaymış hocam; gerçek pozlarda 146 dereceye kadar okunuyor. O yüzden operatör J3 sınırını 155 dereceye çıkardı."]),
    (["5150 hatası ne", "origin has not been set ne demek", "orijin neden kayboldu"],
     ["Hata 5150 orijinin kaybolduğu anlamına gelir hocam; genelde enkoder pilleri zayıflayınca olur. Orijin yeniden ayarlanana kadar hareket komutları reddedilir; pilleri kontrol etmenizi öneririm."]),
    (["koordinat sistemin nasıl", "X Y Z neyi gösteriyor", "yukarı dediğinde hangi eksen"],
     ["Milimetre cinsinden hocam: X ileri, Y sol, Z yukarı. Yukarı-aşağı Z, sol-sağ Y ekseni; ileri-geri ise kolun uzanma yönü."]),
    (["sınıra gelirsen ne yapıyorsun", "limit aşılırsa ne olur"],
     ["Sınıra kadar gidip orada dururum hocam ve ne kadar gidebildiğimi söylerim. Düz yol engelliyse bileği eğerek aynı noktaya başka bir yoldan gitmeyi denerim."]),
    (["hangi şekilleri çizebilirsin", "ne çizebilirsin"],
     ["Kare, dikdörtgen, daire ve yay çizebilirim hocam, yatay ya da dikey. Boyutu santim olarak söylemeniz yeterli."]),
    (["beceri nedir", "bana beceri nasıl öğretirim", "yeni hareket nasıl öğretirim"],
     ["Bir isim ve adımları söylemeniz yeterli hocam, örneğin: 'selam adında bir beceri öğren: bileği 30 derece döndür, sonra geri döndür'. Sonra 'selam yap' dediğinizde yaparım."]),
    (["noktayı nasıl kaydederim", "konum nasıl kaydedilir"],
     ["Kolu istediğiniz yere götürün ve 'bu konumu kutu olarak kaydet' deyin hocam; sonra 'kutuya git' dediğinizde oraya dönerim."]),
]

CONCEPTS = [
    (["TCP nedir", "araç merkez noktası ne demek", "tool center point ne"],
     ["TCP, aracın uç noktası hocam; yukarı, sola gibi düz hareketlerde hareket ettirdiğim nokta bu. Bizde tutucunun flanş merkezi."]),
    (["ters kinematik nedir", "inverse kinematics ne", "eklem açılarını nasıl buluyorsun"],
     ["Ters kinematik, ucun gitmesini istediğimiz konumdan eklem açılarını hesaplamak hocam. Ben 5 eklemin geometrisiyle bunu tam olarak çözüyorum; bazen aynı noktaya birden fazla çözüm olur."]),
    (["ileri kinematik nedir", "forward kinematics ne"],
     ["İleri kinematik, eklem açılarından ucun konumunu hesaplamak hocam. Omuz yüksekliği, kol boyları ve açılarla X, Y, Z buluyorum."]),
    (["tekrarlanabilirlik ile doğruluk farkı ne", "hassasiyet ile doğruluk aynı mı"],
     ["Tekrarlanabilirlik aynı noktaya ne kadar tutarlı döndüğüm, doğruluk ise istenen koordinata ne kadar yakın vardığım hocam. Ben ±0,02 milimetre tekrarlanabilirim; doğruluk kalibrasyona bağlı."]),
    (["servo motor nedir", "servo ne demek"],
     ["Servo motor, konumunu enkoderle ölçüp sürekli düzelten motor hocam. Servo açıkken eklemler yerinde tutulur ve hareket edebilirim; kapalıyken frenler devrededir."]),
    (["enkoder nedir", "mutlak enkoder ne"],
     ["Enkoder eklemin açısını ölçen sensör hocam. Mutlak enkoder, güç kesilse bile konumu hatırlar; bunun için pille beslenir."]),
    (["eklem hareketi ile düz hareket farkı ne", "lineer hareket ne demek"],
     ["Eklem hareketinde her eklem kendi açısına döner, uç eğri bir yol çizer. Düz harekette ise uç düz bir çizgi izler; bunun için eklemleri birlikte hesaplıyorum hocam."]),
    (["tekillik nedir", "singularity ne"],
     ["Tekillik, kolun bazı yönlerde hareket kabiliyetini kaybettiği duruş hocam; örneğin kol tamamen gerildiğinde. Bu noktalarda düz hareket zorlaşır, ben de başka yol denerim."]),
    (["ROS nedir", "ROS 2 ne işe yarıyor"],
     ["ROS 2, robot yazılımlarını birbirine bağlayan bir altyapı hocam. Sürücü, planlayıcı, arayüz ve ben ayrı düğümler olarak onun üzerinden konuşuyoruz."]),
    (["pnömatik nedir", "havalı tutucu nasıl çalışır"],
     ["Pnömatik, basınçlı havayla çalışan sistem hocam. Tutucumun pistonuna hava verilince parmaklar kapanıyor ya da açılıyor."]),
    (["çarpışma kontrolü nasıl yapılıyor", "bir yere çarparsan ne olur"],
     ["MoveIt, planladığı yolu robotun modeli ve tanımlı engellerle karşılaştırıyor hocam; çarpışma varsa planı reddediyor. Ama tanımlanmamış bir cisim varsa göremem, o yüzden çevreye dikkat etmek gerekir."]),
    (["5 eksenli ile 6 eksenli robot farkı ne"],
     ["6 eksenli kol ucu her yöne çevirebilir hocam. Ben 5 eksenliyim; tutucuyu eğip döndürebiliyorum ama yana yatıramıyorum, bu yüzden bazı yönelimler mümkün değil."]),
]

TROUBLE = [
    (["robot hareket etmiyor", "neden hareket etmiyorsun", "komut verdim ama kıpırdamadın", "robot çalışmıyor"],
     "Önce durumuma bakalım hocam: servo kapalı olabilir, bir alarm olabilir ya da orijin kaybolmuş olabilir.", ["STATUS"]),
    (["alarm veriyor ne yapayım", "kontrolörde hata var"],
     "Hemen durumu okuyorum hocam; alarm kodunu görünce ne yapmamız gerektiğini söylerim.", ["STATUS"]),
    (["bağlantı koptu", "robota bağlanamıyorum", "seri port gitti"],
     "USB-seri adaptörün takılı olduğunu kontrol edin hocam; bazen düşüyor. Taktıktan sonra durumu okuyabilirim.", []),
    (["servoyu nasıl açarım", "servo nasıl açılır"],
     "Arayüzdeki Servo ON düğmesine basmanız yeterli hocam; güvenlik gereği bunu ben yapamıyorum.", []),
]

RESULTS = [
    ("5 santim sola git", ["MOVE_TCP LEFT 50"],
     "Tamam hocam. sınır nedeniyle 5 santim yerine 4 milimetre sola gidildi (taban (J1) sınırı).",
     ["neden bu kadar az gittin", "neden 5 santim gitmedin", "niye durdun", "ne oldu"],
     "Taban eklemi (J1) -150 derece sınırına dayandı hocam, o yüzden 4 milimetrede durdum. Başka bir noktadan yaklaşırsak daha fazla gidebilirim."),
    ("20 santim aşağı in", ["MOVE_TCP DOWN 200"],
     "Tamam hocam. düz çizgide dirsek (J3) sınırı engel oluyor; aynı noktaya farklı bir yoldan gittim, bileği +90° eğerek.",
     ["neden bileği eğdin", "bileği niye büküldü", "ne yaptın az önce"],
     "Düz inerken dirsek (J3) sınırına takılacaktım hocam; aynı noktaya ulaşmak için bileği 90 derece eğip başka bir duruşla indim."),
    ("3 santim yukarı çık", ["MOVE_TCP UP 30"],
     "Tamam hocam. araç 30 mm yukarı: Z 382→412.",
     ["az önce ne yaptın", "ne kadar çıktın", "şimdi ne kadar yüksektesin"],
     "3 santim yukarı çıktım hocam; uç 382 milimetreden 412 milimetreye yükseldi."),
    ("8 santimlik bir daire çiz", ["DRAW_CIRCLE 80"],
     "Tamam hocam. önce sığacağı yere 40 mm gidip 8 santim çaplı yatay daire çizdim.",
     ["neden önce kaydın", "daireden önce niye hareket ettin"],
     "Dairenin tamamı bulunduğum yerde sığmıyordu hocam; önce 4 santim kayıp sığacağı yere geçtim, sonra daireyi çizdim."),
    ("yukarı çık", ["MOVE_TCP UP 20"],
     "Hareket durdu hocam; planlayıcının mesajı: tool Z +20 mm: goal rejected (another move running?).",
     ["neden olmadı", "ne oldu", "niye hata verdi"],
     "Planlayıcı hareketi reddetti hocam; o sırada başka bir hareket çalışıyordu. Biraz bekleyip tekrar deneyebiliriz."),
]


def g_knowledge(world):
    q, a = pick(KNOWLEDGE + CONCEPTS)
    req = pick(q)
    return with_frame(req, "tr", question=True), pick(a), []


def g_trouble(world):
    q, a, cmds = pick(TROUBLE)
    return with_frame(pick(q), question=True), a, cmds


def g_explain(world):
    req0, cmds0, result, questions, answer = pick(RESULTS)
    return req0, cmds0, result, pick(questions), answer


def g_pick_place(world):
    names = list(world.points)
    if len(names) < 2:
        return None
    a, c = R.sample(names, 2)
    text = pick([f"{a} noktasındaki parçayı alıp {dative(c)} koy", f"parçayı {a}'dan al {dative(c)} bırak".replace("'dan", "'dan"),
                 f"{a}'daki parçayı {c} noktasına taşı", f"{a} konumundan parçayı al, {c} konumuna götür"])
    cmds = ["GRIPPER OPEN", f"GOTO_POINT {a}", "GRIPPER CLOSE", "WAIT 0.5", f"GOTO_POINT {c}", "GRIPPER OPEN"]
    return with_frame(text), f"Tutucuyu açıp {a} noktasına gidiyorum, parçayı alıp {c} noktasına bırakacağım hocam.", cmds


# -- assembly ------------------------------------------------------------------------------------

GENERATORS = [
    (g_tcp_tr, 22), (g_tcp_en, 5), (g_joint_rel_tr, 7), (g_joint_rel_en, 2), (g_tilt, 4), (g_joint_abs, 3),
    (g_gripper, 5), (g_speed, 3), (g_wait, 1.5), (g_home, 2.5), (g_goto, 5), (g_save, 2), (g_shape, 8),
    (g_shape_en, 1.5), (g_line, 4), (g_read, 5), (g_sequence, 5), (g_skill, 3), (g_servo_conveyor, 2.5),
    (g_chat, 5), (g_mixed, 2), (g_followup, 7), (g_knowledge, 9), (g_trouble, 1.5), (g_explain, 2.5),
    (g_pick_place, 1.5),
]


def check_label(cmds, world):
    """Every command must parse; no servo on, no J4, speed <= 30, wait <= 10,
    known points and skills."""
    for c in cmds:
        c = b.normalize(c)
        name, m = b.match(c)
        if name is None or name == "servo_on":
            return f"bad command {c!r}"
        if name == "speed" and not 1 <= float(m[1]) <= b.MAX_OVERRIDE:
            return c
        if name == "wait" and not 0 < float(m[1]) <= b.MAX_WAIT_S:
            return c
        if name == "goto" and m[1] not in world.points:
            return c
        if name in ("run_skill", "forget") and m[1] not in world.skills:
            return c
        if name in ("jrel", "jabs") and "J4" in c:
            return c
        if name == "define":
            for step in b.split_skill_body(m[2]):
                if b.match(b.normalize(step))[0] is None:
                    return f"bad skill step {step!r}"
    return None


def make_example():
    world = make_world()
    gen = R.choices([g for g, _ in GENERATORS], weights=[w for _, w in GENERATORS])[0]
    out = gen(world)
    if out is None:
        return None
    history = []
    if gen is g_followup:
        (prev_req, prev_say, prev_cmds), req, say, cmds = out
        history = [{"request": prev_req, "say": prev_say, "commands": prev_cmds}]
    elif gen is g_explain:
        prev_req, prev_cmds, result, req, say = out
        history = [{"request": prev_req, "say": "Tamam hocam.", "commands": prev_cmds, "result": result}]
        cmds = []
    else:
        req, say, cmds = out
    if maybe(0.15) and not req.isascii():
        req = noisy(req)
    bad = check_label(cmds, world)
    if bad:
        raise ValueError(f"{gen.__name__}: {req!r} -> {cmds}: {bad}")
    messages = b.build_messages(req, world, b.DEFAULT_LIMITS, None, history=history, compact=True)
    answer = json.dumps({"say": say, "commands": cmds}, ensure_ascii=False)
    return {"messages": messages + [{"role": "assistant", "content": answer}],
            "meta": {"gen": gen.__name__, "request": req, "commands": cmds, "history": history,
                     "points": list(world.points), "skills": list(world.skills)}}


def main():
    global R
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=6000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=os.path.join(HERE, "data", "train.jsonl"))
    args = ap.parse_args()
    R = random.Random(args.seed)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    seen, rows = set(), []
    tries = 0
    while len(rows) < args.n and tries < args.n * 20:
        tries += 1
        ex = make_example()
        if ex is None:
            continue
        key = (ex["meta"]["request"].lower(), json.dumps(ex["meta"]["history"]))
        if key in seen:
            continue
        seen.add(key)
        rows.append(ex)
    with open(args.out, "w") as f:
        for ex in rows:
            f.write(json.dumps(ex, ensure_ascii=False) + "\n")
    counts = {}
    for ex in rows:
        counts[ex["meta"]["gen"]] = counts.get(ex["meta"]["gen"], 0) + 1
    print(f"{len(rows)} examples -> {args.out}")
    print(", ".join(f"{k} {v}" for k, v in sorted(counts.items(), key=lambda kv: -kv[1])))


if __name__ == "__main__":
    main()
