# Transitional attachment-origin VV envelope

Status: development checkpoint, not the final normalized-loading input.

The motion and activation arrays are preserved from the predecessor distal-force envelope. The NPZ wrench_cases_T array retains legacy tibia-origin wrenches; the attachment-origin runtime branch constructs its loads separately from loading_model metadata.

Known limitations:
- The retained force magnitude does not produce the required 10 Nm generalized VV moment at the new attachment locations.
- Attachment-specific normalized forces have been computed separately but are not integrated into this envelope.
- Signed cases are declared as [-1, 1].
- Robot-tibia collision clearance is not established by this input.
- Contact-frame/robot-EE separation and the proposed interface standoff are not implemented in this checkpoint.

Do not use this envelope for the final optimization campaign.

A copied loading_validation_report.json may exist locally. It describes the predecessor loading model, not this transitional input, and is deliberately excluded from the checkpoint.
