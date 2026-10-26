import hashlib,json,tempfile,unittest
from pathlib import Path
from agentic_interaction_v87.gpu_capacity import admission,evidence


class CapacityTests(unittest.TestCase):
    def test_split_keeps_original_combined_budget_and_measured_worker_margin(self):
        r=admission({0:7832,4:6303},0,4,dict(peak_process_memory_MiB=4726))
        self.assertEqual(r['physics_budget_MiB'],7274)
        self.assertEqual(r['physics_budget_MiB']+r['measured_SAM_peak_MiB'],12000)
        self.assertTrue(r['initialized_native_qualification_required'])

    def test_separate_cards_each_require_adequate_capacity(self):
        for free in [{0:3500,4:20000},{0:20000,4:5000}]:
            with self.assertRaises(ValueError):admission(free,0,4,dict(peak_process_memory_MiB=4726))
        with self.assertRaises(ValueError):admission({0:20000},0,0,dict(peak_process_memory_MiB=4726))

    def test_smaller_card_only_grants_no_load_qualification_not_interaction(self):
        r=admission({7:5217,4:5841},7,4,dict(peak_process_memory_MiB=4726))
        self.assertFalse(r['interaction_authorized_before_native_measurement'])
        self.assertEqual(r['physics_budget_MiB'],7274)

    def test_worker_evidence_binds_hash_gpu_checkpoint_and_real_success(self):
        with tempfile.TemporaryDirectory()as folder:
            p=Path(folder)/'measurement.json'
            d=dict(returncode=0,original_worker_unmodified=True,gpu=4,
                checkpoint_sha256='matched',image_resolution=[1280,960],peak_process_memory_MiB=4726)
            raw=json.dumps(d).encode();p.write_bytes(raw)
            r=dict(path=str(p),sha256=hashlib.sha256(raw).hexdigest())
            self.assertEqual(evidence(r,'matched',4)['returncode'],0)
            for checkpoint,gpu in [('different',4),('matched',0)]:
                with self.assertRaises(ValueError):evidence(r,checkpoint,gpu)
            p.write_text(json.dumps(dict(d,returncode=1)))
            with self.assertRaises(ValueError):evidence(r,'matched',4)


if __name__=='__main__':unittest.main()
