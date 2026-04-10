use std::collections::BTreeMap;

pub type ConfigKey = String;
pub type FieldPath = String;
pub type LayerPath = String;
pub type ProvenanceMap = BTreeMap<FieldPath, LayerPath>;
