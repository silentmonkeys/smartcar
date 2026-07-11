from functools import lru_cache
from config.constants import SIMILARITY_THRESHOLD, CONFIDENCE_THRESHOLD


@lru_cache(maxsize=128)
def levenshtein_distance(s1: str, s2: str) -> int:
    if len(s1) < len(s2):
        return levenshtein_distance(s2, s1)
    if len(s2) == 0:
        return len(s1)
    prev_row = range(len(s2) + 1)
    for i, c1 in enumerate(s1):
        curr_row = [i + 1]
        for j, c2 in enumerate(s2):
            insertions = prev_row[j + 1] + 1
            deletions = curr_row[j] + 1
            substitutions = prev_row[j] + (c1 != c2)
            curr_row.append(min(insertions, deletions, substitutions))
        prev_row = curr_row
    return prev_row[-1]


def text_similarity(s1, s2):
    if not s1 or not s2:
        return 0.0
    max_len = max(len(s1), len(s2))
    if max_len == 0:
        return 1.0
    return 1.0 - levenshtein_distance(s1, s2) / max_len


def get_consensus_result(history):
    if not history:
        return None

    clusters = {}
    for text, conf in history:
        merged = False
        for key in list(clusters.keys()):
            if text_similarity(text, key) >= SIMILARITY_THRESHOLD:
                clusters[key]["count"] += 1
                clusters[key]["total_conf"] += conf
                merged = True
                break
        if not merged:
            clusters[text] = {"count": 1, "total_conf": conf}

    best_text, best_score = None, 0.0
    total = len(history)
    for text, data in clusters.items():
        stability = data["count"] / total
        avg_conf = data["total_conf"] / data["count"]
        score = stability * 0.6 + avg_conf * 0.4
        if score > best_score:
            best_score = score
            best_text = text
    return best_text