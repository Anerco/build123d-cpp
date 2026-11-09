// The bd_materials catalog as far as build123d uses it: each name resolve() accepts and
// the physical material it stands for. Generated from bd_materials 0.2.5's registry
// (family names give their default grade).
#include <algorithm>
#include <cctype>
#include <map>
#include <stdexcept>

#include "build123d/materials.hpp"

namespace build123d {

namespace {

const std::map<std::string, RangeMaterial>& catalog() {
    static const std::map<std::string, RangeMaterial> table = {
    {"abs", {"ABS_GENERIC", 1050.0, "plastic", "ABS", false}},
    {"abs_flame_retardant", {"ABS_FLAME_RETARDANT", 1200.0, "plastic", "ABS", false}},
    {"abs_generic", {"ABS_GENERIC", 1050.0, "plastic", "ABS", false}},
    {"alloy_steel", {"AlloySteel_G4140_QUENCHED_TEMPERED", 7850.0, "metal", "alloy_steel", false}},
    {"alloysteel.g52100_hardened_low_tempered", {"AlloySteel.G52100_HARDENED_LOW_TEMPERED", 7810.0, "metal", "alloy_steel", false}},
    {"alloysteel_g1215_cold_drawn", {"AlloySteel_G1215_COLD_DRAWN", 7850.0, "metal", "alloy_steel", false}},
    {"alloysteel_g4140_quenched_tempered", {"AlloySteel_G4140_QUENCHED_TEMPERED", 7850.0, "metal", "alloy_steel", false}},
    {"alloysteel_g4340_quenched_tempered", {"AlloySteel_G4340_QUENCHED_TEMPERED", 7850.0, "metal", "alloy_steel", false}},
    {"alu_alsi10mg_as_built", {"Alu_ALSI10MG_AS_BUILT", 2670.0, "metal", "aluminum", false}},
    {"alu_g2024_aerospace", {"Alu_G2024_AEROSPACE", 2780.0, "metal", "aluminum", false}},
    {"alu_g2a12_t4", {"Alu_G2A12_T4", 2800.0, "metal", "aluminum", false}},
    {"alu_g5052_h32", {"Alu_G5052_H32", 2680.0, "metal", "aluminum", false}},
    {"alu_g6061_t6", {"Alu_G6061_T6", 2700.0, "metal", "aluminum", false}},
    {"alu_g7075_t6", {"Alu_G7075_T6", 2810.0, "metal", "aluminum", false}},
    {"aluminum", {"Alu_G6061_T6", 2700.0, "metal", "aluminum", false}},
    {"american_cherry", {"Hardwood_AMERICAN_CHERRY", 570.0, "wood", "cherry", false}},
    {"asa", {"Asa_GENERIC", 1070.0, "plastic", "ASA", false}},
    {"asa_generic", {"Asa_GENERIC", 1070.0, "plastic", "ASA", false}},
    {"ash", {"Hardwood_ASH", 670.0, "wood", "ash", false}},
    {"beech", {"Hardwood_BEECH", 720.0, "wood", "beech", false}},
    {"birch", {"Hardwood_BIRCH", 660.0, "wood", "birch", false}},
    {"borosilicate", {"Borosilicate_GENERIC", 2230.0, "glass", "borosilicate", true}},
    {"borosilicate_generic", {"Borosilicate_GENERIC", 2230.0, "glass", "borosilicate", true}},
    {"brass", {"Brass_C360_HALF_HARD", 8500.0, "metal", "brass", false}},
    {"brass_c360_half_hard", {"Brass_C360_HALF_HARD", 8500.0, "metal", "brass", false}},
    {"brass_c360_soft_annealed", {"Brass_C360_SOFT_ANNEALED", 8500.0, "metal", "brass", false}},
    {"bronze", {"Bronze_C51000_PHOSPHOR", 8800.0, "metal", "bronze", false}},
    {"bronze_c51000_phosphor", {"Bronze_C51000_PHOSPHOR", 8800.0, "metal", "bronze", false}},
    {"bronze_c90500_tin", {"Bronze_C90500_TIN", 8800.0, "metal", "bronze", false}},
    {"bronze_c93200_bearing", {"Bronze_C93200_BEARING", 8900.0, "metal", "bronze", false}},
    {"cardboard", {"Cardboard_CORRUGATED", 140.0, "paper", "corrugated_cardboard", false}},
    {"cardboard_corrugated", {"Cardboard_CORRUGATED", 140.0, "paper", "corrugated_cardboard", false}},
    {"castable", {"Castable_GENERIC", 1150.0, "resin", "resin", false}},
    {"castable_generic", {"Castable_GENERIC", 1150.0, "resin", "resin", false}},
    {"ceramic", {"Ceramic_GENERIC", 1650.0, "resin", "resin", false}},
    {"ceramic_generic", {"Ceramic_GENERIC", 1650.0, "resin", "resin", false}},
    {"cfrp", {"CFRP_PLATE", 1600.0, "plastic", "CFRP", false}},
    {"cfrp_plate", {"CFRP_PLATE", 1600.0, "plastic", "CFRP", false}},
    {"copper", {"Copper_C110_ANNEALED", 8900.0, "metal", "copper", false}},
    {"copper_c110_annealed", {"Copper_C110_ANNEALED", 8900.0, "metal", "copper", false}},
    {"engineered_wood", {"EngineeredWood_MDF", 750.0, "wood", "mdf", false}},
    {"engineeredwood_mdf", {"EngineeredWood_MDF", 750.0, "wood", "mdf", false}},
    {"engineeredwood_osb", {"EngineeredWood_OSB", 640.0, "wood", "osb", false}},
    {"esd", {"Esd_GENERIC", 1300.0, "resin", "resin", false}},
    {"esd_generic", {"Esd_GENERIC", 1300.0, "resin", "resin", false}},
    {"felt", {"Felt_GENERIC", 120.0, "textile", "felt", false}},
    {"felt_generic", {"Felt_GENERIC", 120.0, "textile", "felt", false}},
    {"flexible", {"Flexible_GENERIC", 1150.0, "resin", "resin", false}},
    {"flexible_generic", {"Flexible_GENERIC", 1150.0, "resin", "resin", false}},
    {"foamboard", {"Foamboard_GENERIC", 120.0, "paper", "foamboard", false}},
    {"foamboard_generic", {"Foamboard_GENERIC", 120.0, "paper", "foamboard", false}},
    {"fr4", {"FR4_GENERIC", 1850.0, "plastic", "FR4", false}},
    {"fr4_generic", {"FR4_GENERIC", 1850.0, "plastic", "FR4", false}},
    {"gold", {"Gold_PURE", 19300.0, "metal", "gold", false}},
    {"gold_pure", {"Gold_PURE", 19300.0, "metal", "gold", false}},
    {"hardwood", {"Hardwood_GENERIC", 675.0, "wood", "oak", false}},
    {"hardwood_american_cherry", {"Hardwood_AMERICAN_CHERRY", 570.0, "wood", "cherry", false}},
    {"hardwood_ash", {"Hardwood_ASH", 670.0, "wood", "ash", false}},
    {"hardwood_beech", {"Hardwood_BEECH", 720.0, "wood", "beech", false}},
    {"hardwood_birch", {"Hardwood_BIRCH", 660.0, "wood", "birch", false}},
    {"hardwood_generic", {"Hardwood_GENERIC", 675.0, "wood", "oak", false}},
    {"hardwood_maple", {"Hardwood_MAPLE", 705.0, "wood", "maple", false}},
    {"hardwood_oak", {"Hardwood_OAK", 755.0, "wood", "oak", false}},
    {"hardwood_walnut", {"Hardwood_WALNUT", 640.0, "wood", "walnut", false}},
    {"high_temp", {"HighTemp_GENERIC", 1300.0, "resin", "resin", false}},
    {"hightemp_generic", {"HighTemp_GENERIC", 1300.0, "resin", "resin", false}},
    {"leather", {"Leather_GENERIC", 950.0, "textile", "leather", false}},
    {"leather_generic", {"Leather_GENERIC", 950.0, "textile", "leather", false}},
    {"magnesium", {"Magnesium_GENERIC_STRUCTURAL", 1810.0, "metal", "magnesium", false}},
    {"magnesium_generic_structural", {"Magnesium_GENERIC_STRUCTURAL", 1810.0, "metal", "magnesium", false}},
    {"maple", {"Hardwood_MAPLE", 705.0, "wood", "maple", false}},
    {"mdf", {"EngineeredWood_MDF", 750.0, "wood", "mdf", false}},
    {"mild_steel", {"MildSteel_PLAIN_CARBON_GENERIC", 7800.0, "metal", "mild_steel", false}},
    {"mildsteel_g1018_cold_drawn", {"MildSteel_G1018_COLD_DRAWN", 7850.0, "metal", "mild_steel", false}},
    {"mildsteel_g1045_cold_drawn", {"MildSteel_G1045_COLD_DRAWN", 7850.0, "metal", "mild_steel", false}},
    {"mildsteel_ga36_hot_rolled", {"MildSteel_GA36_HOT_ROLLED", 7850.0, "metal", "mild_steel", false}},
    {"mildsteel_plain_carbon_generic", {"MildSteel_PLAIN_CARBON_GENERIC", 7800.0, "metal", "mild_steel", false}},
    {"nylon", {"Nylon_PA12", 1010.0, "plastic", "PA", false}},
    {"nylon_pa12", {"Nylon_PA12", 1010.0, "plastic", "PA", false}},
    {"nylon_pa12_gf35", {"Nylon_PA12_GF35", 1300.0, "plastic", "PA", false}},
    {"nylon_pa12_sintered", {"Nylon_PA12_SINTERED", 1000.0, "plastic", "PA", false}},
    {"nylon_pa6", {"Nylon_PA6", 1140.0, "plastic", "PA", false}},
    {"oak", {"Hardwood_OAK", 755.0, "wood", "oak", false}},
    {"osb", {"EngineeredWood_OSB", 640.0, "wood", "osb", false}},
    {"paper", {"Paper_OFFICE", 800.0, "paper", "paper", false}},
    {"paper_office", {"Paper_OFFICE", 800.0, "paper", "paper", false}},
    {"pc", {"PC_GENERIC", 1200.0, "plastic", "PC", true}},
    {"pc_generic", {"PC_GENERIC", 1200.0, "plastic", "PC", true}},
    {"pe", {"PE_HDPE", 960.0, "plastic", "PE", false}},
    {"pe_hdpe", {"PE_HDPE", 960.0, "plastic", "PE", false}},
    {"peek", {"Peek_MOLDED", 1300.0, "plastic", "PEEK", false}},
    {"peek_molded", {"Peek_MOLDED", 1300.0, "plastic", "PEEK", false}},
    {"peek_printed", {"Peek_PRINTED", 1300.0, "plastic", "PEEK", false}},
    {"petg", {"PETG_GENERIC", 1270.0, "plastic", "PETG", false}},
    {"petg_cf", {"PETG_CF", 1320.0, "plastic", "PETG", false}},
    {"petg_generic", {"PETG_GENERIC", 1270.0, "plastic", "PETG", false}},
    {"phenolic", {"Phenolic_BAKELITE", 1300.0, "plastic", "phenolic", false}},
    {"phenolic_bakelite", {"Phenolic_BAKELITE", 1300.0, "plastic", "phenolic", false}},
    {"pine", {"Softwood_PINE", 500.0, "wood", "pine", false}},
    {"pla", {"PLA_GENERIC", 1240.0, "plastic", "PLA", false}},
    {"pla_carbon_filled", {"PLA_CARBON_FILLED", 1250.0, "plastic", "PLA", false}},
    {"pla_generic", {"PLA_GENERIC", 1240.0, "plastic", "PLA", false}},
    {"pmma", {"PMMA_GENERIC", 1180.0, "plastic", "PMMA", true}},
    {"pmma_generic", {"PMMA_GENERIC", 1180.0, "plastic", "PMMA", true}},
    {"pom", {"POM_GENERIC", 1410.0, "plastic", "POM", false}},
    {"pom_generic", {"POM_GENERIC", 1410.0, "plastic", "POM", false}},
    {"pp", {"PP_GENERIC", 905.0, "plastic", "PP", false}},
    {"pp_generic", {"PP_GENERIC", 905.0, "plastic", "PP", false}},
    {"pps", {"PPS_CF", 1400.0, "plastic", "PPS", false}},
    {"pps_cf", {"PPS_CF", 1400.0, "plastic", "PPS", false}},
    {"ptfe", {"PTFE_GENERIC", 2170.0, "plastic", "PTFE", false}},
    {"ptfe_generic", {"PTFE_GENERIC", 2170.0, "plastic", "PTFE", false}},
    {"rubber", {"Rubber_GENERIC", 1200.0, "plastic", "rubber", false}},
    {"rubber_generic", {"Rubber_GENERIC", 1200.0, "plastic", "rubber", false}},
    {"silver", {"Silver_PURE", 10490.0, "metal", "silver", false}},
    {"silver_pure", {"Silver_PURE", 10490.0, "metal", "silver", false}},
    {"soda_lime", {"SodaLime_GENERIC", 2500.0, "glass", "soda_lime", true}},
    {"sodalime_generic", {"SodaLime_GENERIC", 2500.0, "glass", "soda_lime", true}},
    {"softwood", {"Softwood_GENERIC", 470.0, "wood", "spruce", false}},
    {"softwood_generic", {"Softwood_GENERIC", 470.0, "wood", "spruce", false}},
    {"softwood_pine", {"Softwood_PINE", 500.0, "wood", "pine", false}},
    {"softwood_spruce", {"Softwood_SPRUCE", 430.0, "wood", "spruce", false}},
    {"spring_steel", {"SpringSteel_GENERIC_QUENCHED_TEMPERED", 7850.0, "metal", "spring_steel", false}},
    {"springsteel_generic_quenched_tempered", {"SpringSteel_GENERIC_QUENCHED_TEMPERED", 7850.0, "metal", "spring_steel", false}},
    {"spruce", {"Softwood_SPRUCE", 430.0, "wood", "spruce", false}},
    {"stainless", {"Stainless_G304_ANNEALED", 7930.0, "metal", "stainless", false}},
    {"stainless_g201_annealed", {"Stainless_G201_ANNEALED", 7860.0, "metal", "stainless", false}},
    {"stainless_g303_annealed", {"Stainless_G303_ANNEALED", 8030.0, "metal", "stainless", false}},
    {"stainless_g304_annealed", {"Stainless_G304_ANNEALED", 7930.0, "metal", "stainless", false}},
    {"stainless_g316l_annealed", {"Stainless_G316L_ANNEALED", 8000.0, "metal", "stainless", false}},
    {"stainless_g316l_as_built", {"Stainless_G316L_AS_BUILT", 7990.0, "metal", "stainless", false}},
    {"stainless_g430_annealed", {"Stainless_G430_ANNEALED", 7750.0, "metal", "stainless", false}},
    {"stainless_g430_cold_worked", {"Stainless_G430_COLD_WORKED", 7750.0, "metal", "stainless", false}},
    {"standard", {"Standard_GENERIC", 1150.0, "resin", "resin", false}},
    {"standard_generic", {"Standard_GENERIC", 1150.0, "resin", "resin", false}},
    {"titanium", {"Titanium_GR5_ANNEALED", 4430.0, "metal", "titanium", false}},
    {"titanium_gr5_annealed", {"Titanium_GR5_ANNEALED", 4430.0, "metal", "titanium", false}},
    {"titanium_tc4_as_built", {"Titanium_TC4_AS_BUILT", 4430.0, "metal", "titanium", false}},
    {"tool_steel", {"ToolSteel_D2_HARDENED", 7700.0, "metal", "tool_steel", false}},
    {"toolsteel_a2_hardened", {"ToolSteel_A2_HARDENED", 7860.0, "metal", "tool_steel", false}},
    {"toolsteel_a3_hardened", {"ToolSteel_A3_HARDENED", 7860.0, "metal", "tool_steel", false}},
    {"toolsteel_d2_hardened", {"ToolSteel_D2_HARDENED", 7700.0, "metal", "tool_steel", false}},
    {"toolsteel_generic_as_built", {"ToolSteel_GENERIC_AS_BUILT", 7800.0, "metal", "tool_steel", false}},
    {"toolsteel_h13_hardened", {"ToolSteel_H13_HARDENED", 7800.0, "metal", "tool_steel", false}},
    {"toolsteel_o1_hardened", {"ToolSteel_O1_HARDENED", 7830.0, "metal", "tool_steel", false}},
    {"toolsteel_s7_hardened", {"ToolSteel_S7_HARDENED", 7830.0, "metal", "tool_steel", false}},
    {"tough", {"Tough_GENERIC", 1150.0, "resin", "resin", false}},
    {"tough_generic", {"Tough_GENERIC", 1150.0, "resin", "resin", false}},
    {"tpu", {"TPU_SHORE_95A", 1200.0, "plastic", "TPU", false}},
    {"tpu_shore_95a", {"TPU_SHORE_95A", 1200.0, "plastic", "TPU", false}},
    {"tpu_sintered", {"TPU_SINTERED", 1200.0, "plastic", "TPU", false}},
    {"transparent", {"Transparent_GENERIC", 1300.0, "resin", "resin", true}},
    {"transparent_generic", {"Transparent_GENERIC", 1300.0, "resin", "resin", true}},
    {"walnut", {"Hardwood_WALNUT", 640.0, "wood", "walnut", false}},
    {"woven", {"Woven_GENERIC", 500.0, "textile", "fabric_weave", false}},
    {"woven_generic", {"Woven_GENERIC", 500.0, "textile", "fabric_weave", false}},
    };
    return table;
}

/// Fold a user-supplied name to its registry key (case/space/hyphen-insensitive).
std::string normalize(const std::string& name) {
    const auto first = name.find_first_not_of(" \t\n\r\f\v");
    const auto last = name.find_last_not_of(" \t\n\r\f\v");
    std::string key = first == std::string::npos ? "" : name.substr(first, last - first + 1);
    for (char& c : key) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (c == ' ' || c == '-') c = '_';
    }
    return key;
}

}  // namespace

FinishedMaterial resolve_material(const std::string& name) {
    const auto& table = catalog();
    const auto it = table.find(normalize(name));
    if (it == table.end()) {
        throw std::invalid_argument("unknown material '" + name + "' (material_names() lists all " +
                                    std::to_string(table.size()) + ")");
    }
    return FinishedMaterial(it->second);
}

FinishedMaterial::FinishedMaterial(const std::string& name) : FinishedMaterial(resolve_material(name)) {}

std::vector<std::string> material_names() {
    std::vector<std::string> names;
    for (const auto& entry : catalog()) names.push_back(entry.first);
    return names;
}

}  // namespace build123d
