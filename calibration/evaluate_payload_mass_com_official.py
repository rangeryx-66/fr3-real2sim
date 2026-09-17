"""CLI wrapper for the Scalable Real2Sim mass/CoM-only FR3 adapter.

The existing ``evaluate_payload_id_v2.py`` remains unchanged for regression.
This entry point writes a separate ``payload_id_mass_com_official.json`` audit
file and never modifies the grasp, scan, reconstruction, collision, or USD
stages.
"""
from __future__ import annotations

from real2sim.payload_id_official_mass_com import main


if __name__ == "__main__":
    main()
