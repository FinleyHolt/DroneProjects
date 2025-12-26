/**
 * ELK Reasoner Benchmark for UAV Domain Ontology
 *
 * This Java program loads an OWL ontology, runs ELK classification,
 * performs inference queries, and outputs timing results in JSON format.
 *
 * Enhanced with 100-iteration benchmarking and statistical percentile calculation.
 *
 * Usage:
 *   java ElkBenchmark <ontology-path> <output-json-path> [iterations]
 *
 * Example:
 *   java ElkBenchmark uav_domain.owl results.json 100
 *
 * Author: Finley Holt
 * Date: 2024-12-25
 */

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.reasoner.*;
import org.semanticweb.elk.owlapi.ElkReasonerFactory;

import java.io.*;
import java.util.*;
import java.time.Instant;

public class ElkBenchmark {

    private static OWLOntologyManager manager;
    private static OWLOntology ontology;
    private static OWLReasonerFactory reasonerFactory;
    private static OWLReasoner reasoner;
    private static OWLDataFactory dataFactory;

    // Configuration
    private static int NUM_ITERATIONS = 100;
    private static int WARMUP_ITERATIONS = 5;

    // Timing results
    private static Map<String, Object> results = new LinkedHashMap<>();
    private static List<Map<String, Object>> queryResults = new ArrayList<>();

    // Track all iteration timings for statistical analysis
    private static List<Double> coldStartTimes = new ArrayList<>();
    private static List<Double> classificationTimes = new ArrayList<>();
    private static Map<String, List<Double>> queryTimings = new LinkedHashMap<>();

    public static void main(String[] args) {
        if (args.length < 2) {
            System.err.println("Usage: java ElkBenchmark <ontology-path> <output-json-path> [iterations]");
            System.exit(1);
        }

        String ontologyPath = args[0];
        String outputPath = args[1];

        if (args.length >= 3) {
            NUM_ITERATIONS = Integer.parseInt(args[2]);
        }

        results.put("reasoner", "ELK");
        results.put("reasoner_version", "0.6.0");
        results.put("owl_profile", "OWL 2 EL");
        results.put("timestamp", Instant.now().toString());
        results.put("ontology_file", ontologyPath);
        results.put("num_iterations", NUM_ITERATIONS);
        results.put("warmup_iterations", WARMUP_ITERATIONS);

        try {
            System.out.println("=== ELK Reasoner Benchmark ===");
            System.out.println("Iterations: " + NUM_ITERATIONS);
            System.out.println("Warmup: " + WARMUP_ITERATIONS);
            System.out.println();

            // Phase 1: Cold start measurement (first load without JIT warmup)
            System.out.println("Phase 1: Cold start measurement...");
            long coldStartBegin = System.nanoTime();
            loadOntology(ontologyPath);
            createReasoner();
            reasoner.precomputeInferences(InferenceType.CLASS_HIERARCHY);
            long coldStartTime = System.nanoTime() - coldStartBegin;
            results.put("cold_start_time_ms", coldStartTime / 1_000_000.0);
            System.out.println("  Cold start: " + String.format("%.2f", coldStartTime / 1_000_000.0) + " ms");

            // Check consistency
            boolean isConsistent = reasoner.isConsistent();
            results.put("is_consistent", isConsistent);

            if (!isConsistent) {
                System.err.println("WARNING: Ontology is inconsistent!");
            }

            // Ontology metrics (only need once)
            Map<String, Object> metrics = new LinkedHashMap<>();
            metrics.put("class_count", ontology.getClassesInSignature().size());
            metrics.put("object_property_count", ontology.getObjectPropertiesInSignature().size());
            metrics.put("data_property_count", ontology.getDataPropertiesInSignature().size());
            metrics.put("individual_count", ontology.getIndividualsInSignature().size());
            metrics.put("axiom_count", ontology.getAxiomCount());
            results.put("ontology_metrics", metrics);

            // Phase 2: Warmup iterations (JVM JIT compilation)
            System.out.println("\nPhase 2: JVM warmup (" + WARMUP_ITERATIONS + " iterations)...");
            for (int i = 0; i < WARMUP_ITERATIONS; i++) {
                reasoner.dispose();
                manager = null;
                ontology = null;
                reasoner = null;
                System.gc();

                loadOntology(ontologyPath);
                createReasoner();
                reasoner.precomputeInferences(InferenceType.CLASS_HIERARCHY);
                runQueriesWithoutRecording();
                System.out.print(".");
            }
            System.out.println(" done");

            // Phase 3: Classification benchmark (100 iterations)
            System.out.println("\nPhase 3: Classification benchmark (" + NUM_ITERATIONS + " iterations)...");
            for (int i = 0; i < NUM_ITERATIONS; i++) {
                reasoner.dispose();
                manager = null;
                ontology = null;
                reasoner = null;

                // Force GC to reduce variance
                if (i % 10 == 0) {
                    System.gc();
                    Thread.sleep(10);
                }

                loadOntology(ontologyPath);

                long reasonerStart = System.nanoTime();
                createReasoner();
                long reasonerInitTime = System.nanoTime() - reasonerStart;

                long classifyStart = System.nanoTime();
                reasoner.precomputeInferences(InferenceType.CLASS_HIERARCHY);
                long classifyTime = System.nanoTime() - classifyStart;

                coldStartTimes.add((reasonerInitTime + classifyTime) / 1_000_000.0);
                classificationTimes.add(classifyTime / 1_000_000.0);

                if ((i + 1) % 10 == 0) {
                    System.out.println("  " + (i + 1) + "/" + NUM_ITERATIONS + " iterations complete");
                }
            }

            // Phase 4: Query benchmarks (100 iterations each)
            System.out.println("\nPhase 4: Query benchmark (" + NUM_ITERATIONS + " iterations per query)...");
            runBenchmarkQueriesWithIterations();
            results.put("queries", queryResults);

            // Phase 5: Calculate statistics
            System.out.println("\nPhase 5: Computing statistics...");
            Map<String, Object> classificationStats = calculateStats(classificationTimes);
            results.put("classification_stats", classificationStats);

            Map<String, Object> coldStartStats = calculateStats(coldStartTimes);
            results.put("cold_start_stats", coldStartStats);

            // Calculate summary statistics
            Map<String, Object> summary = new LinkedHashMap<>();
            summary.put("total_query_types", queryResults.size());
            summary.put("iterations_per_query", NUM_ITERATIONS);
            results.put("summary", summary);

            // Document ELK limitations
            List<String> limitations = new ArrayList<>();
            limitations.add("ELK only supports OWL 2 EL profile - no negation, disjunction, or universal restrictions");
            limitations.add("Cannot express safety axioms with numeric comparisons (e.g., battery < 20%)");
            limitations.add("Cannot express conditional rules like 'IF battery low THEN return to launch'");
            limitations.add("No support for SWRL rules or datatype reasoning beyond simple assertions");
            limitations.add("Cannot query for violations of numeric thresholds at runtime");
            results.put("elk_limitations", limitations);

            // Phase 6: Dispose and write
            reasoner.dispose();
            writeResults(outputPath);

            System.out.println("\nBenchmark complete. Results written to: " + outputPath);
            System.out.println("\n=== Summary ===");
            System.out.println("Classification p50: " + String.format("%.3f", (Double) classificationStats.get("p50_ms")) + " ms");
            System.out.println("Classification p95: " + String.format("%.3f", (Double) classificationStats.get("p95_ms")) + " ms");
            System.out.println("Classification p99: " + String.format("%.3f", (Double) classificationStats.get("p99_ms")) + " ms");

        } catch (Exception e) {
            results.put("error", e.getMessage());
            e.printStackTrace();
            try {
                writeResults(outputPath);
            } catch (IOException ioe) {
                ioe.printStackTrace();
            }
            System.exit(1);
        }
    }

    private static void loadOntology(String path) throws OWLOntologyCreationException {
        manager = OWLManager.createOWLOntologyManager();
        dataFactory = manager.getOWLDataFactory();

        File ontologyFile = new File(path);
        ontology = manager.loadOntologyFromOntologyDocument(ontologyFile);

        System.out.println("Loaded ontology: " + ontology.getOntologyID());
    }

    private static void createReasoner() {
        reasonerFactory = new ElkReasonerFactory();
        reasoner = reasonerFactory.createReasoner(ontology);
    }

    private static void runQueriesWithoutRecording() {
        String baseIRI = "http://flyby-robotics.com/ontology/uav#";
        OWLClass uav = dataFactory.getOWLClass(IRI.create(baseIRI + "UAV"));
        reasoner.getSubClasses(uav, false);
        reasoner.getSuperClasses(uav, false);
        reasoner.getInstances(dataFactory.getOWLClass(IRI.create(baseIRI + "FlightPhase")), false);
    }

    private static void runBenchmarkQueriesWithIterations() {
        String baseIRI = "http://flyby-robotics.com/ontology/uav#";

        // Define all query specs
        String[][] subclassQueries = {
            {"Q1_UAV_Subclasses", "UAV"},
            {"Q2_Multirotor_Subclasses", "Multirotor"},
            {"Q3_Mission_Subclasses", "UAVMission"},
            {"Q4_Sensor_Subclasses", "UAVSensor"},
            {"Q5_Action_Subclasses", "UAVAction"},
            {"Q6_Airspace_Subclasses", "AirspaceRegion"},
            {"Q15_HoverCapable_Subclasses", "HoverCapable"}
        };

        String[][] subsumptionQueries = {
            {"Q7_FlybyF11_Quadcopter", "FlybyF11", "Quadcopter"},
            {"Q8_Quadcopter_UAV", "Quadcopter", "UAV"}
        };

        String[][] superclassQueries = {
            {"Q9_FlybyF11_Superclasses", "FlybyF11"}
        };

        String[][] instanceQueries = {
            {"Q10_FlightPhase_Instances", "FlightPhase"},
            {"Q11_MissionStatus_Instances", "MissionStatus"},
            {"Q12_Emergency_Instances", "EmergencyCondition"}
        };

        // Run subclass queries with iterations
        for (String[] q : subclassQueries) {
            runSubclassQueryWithIterations(q[0], q[1], baseIRI);
        }

        // Run subsumption queries with iterations
        for (String[] q : subsumptionQueries) {
            runSubsumptionQueryWithIterations(q[0], q[1], q[2], baseIRI);
        }

        // Run superclass queries with iterations
        for (String[] q : superclassQueries) {
            runSuperclassQueryWithIterations(q[0], q[1], baseIRI);
        }

        // Run instance queries with iterations
        for (String[] q : instanceQueries) {
            runInstanceQueryWithIterations(q[0], q[1], baseIRI);
        }

        // Run unsatisfiable query with iterations
        runUnsatisfiableQueryWithIterations("Q13_Unsatisfiable_Classes");

        // Run taxonomy depth query with iterations
        runTaxonomyDepthQueryWithIterations("Q14_UAV_Taxonomy_Depth", "UAV", baseIRI);
    }

    private static void runSubclassQueryWithIterations(String queryId, String className, String baseIRI) {
        List<Double> times = new ArrayList<>();
        Set<String> subClassNames = new HashSet<>();

        OWLClass owlClass = dataFactory.getOWLClass(IRI.create(baseIRI + className));

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            long startTime = System.nanoTime();
            NodeSet<OWLClass> subClasses = reasoner.getSubClasses(owlClass, false);
            long endTime = System.nanoTime();
            times.add((endTime - startTime) / 1_000_000.0);

            // Capture results on first iteration only
            if (i == 0) {
                for (Node<OWLClass> node : subClasses) {
                    for (OWLClass cls : node.getEntities()) {
                        if (!cls.isBottomEntity()) {
                            subClassNames.add(cls.getIRI().getShortForm());
                        }
                    }
                }
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "subclass");
        result.put("target_class", className);
        result.put("result_count", subClassNames.size());
        result.put("results", new ArrayList<>(subClassNames));
        result.put("timing_stats", calculateStats(times));
        queryTimings.put(queryId, times);
        queryResults.add(result);

        System.out.println("  " + queryId + ": p50=" + String.format("%.3f", calculatePercentile(times, 50)) + "ms");
    }

    private static void runSuperclassQueryWithIterations(String queryId, String className, String baseIRI) {
        List<Double> times = new ArrayList<>();
        Set<String> superClassNames = new HashSet<>();

        OWLClass owlClass = dataFactory.getOWLClass(IRI.create(baseIRI + className));

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            long startTime = System.nanoTime();
            NodeSet<OWLClass> superClasses = reasoner.getSuperClasses(owlClass, false);
            long endTime = System.nanoTime();
            times.add((endTime - startTime) / 1_000_000.0);

            if (i == 0) {
                for (Node<OWLClass> node : superClasses) {
                    for (OWLClass cls : node.getEntities()) {
                        if (!cls.isTopEntity()) {
                            superClassNames.add(cls.getIRI().getShortForm());
                        }
                    }
                }
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "superclass");
        result.put("target_class", className);
        result.put("result_count", superClassNames.size());
        result.put("results", new ArrayList<>(superClassNames));
        result.put("timing_stats", calculateStats(times));
        queryTimings.put(queryId, times);
        queryResults.add(result);

        System.out.println("  " + queryId + ": p50=" + String.format("%.3f", calculatePercentile(times, 50)) + "ms");
    }

    private static void runSubsumptionQueryWithIterations(String queryId, String subClass, String superClass, String baseIRI) {
        List<Double> times = new ArrayList<>();
        boolean isSubsumed = false;

        OWLClass sub = dataFactory.getOWLClass(IRI.create(baseIRI + subClass));
        OWLClass sup = dataFactory.getOWLClass(IRI.create(baseIRI + superClass));

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            long startTime = System.nanoTime();
            NodeSet<OWLClass> superClasses = reasoner.getSuperClasses(sub, false);
            boolean found = false;
            for (Node<OWLClass> node : superClasses) {
                if (node.contains(sup)) {
                    found = true;
                    break;
                }
            }
            long endTime = System.nanoTime();
            times.add((endTime - startTime) / 1_000_000.0);

            if (i == 0) {
                isSubsumed = found;
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "subsumption");
        result.put("subclass", subClass);
        result.put("superclass", superClass);
        result.put("is_subsumed", isSubsumed);
        result.put("timing_stats", calculateStats(times));
        queryTimings.put(queryId, times);
        queryResults.add(result);

        System.out.println("  " + queryId + ": p50=" + String.format("%.3f", calculatePercentile(times, 50)) + "ms");
    }

    private static void runInstanceQueryWithIterations(String queryId, String className, String baseIRI) {
        List<Double> times = new ArrayList<>();
        Set<String> instanceNames = new HashSet<>();

        OWLClass owlClass = dataFactory.getOWLClass(IRI.create(baseIRI + className));

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            long startTime = System.nanoTime();
            NodeSet<OWLNamedIndividual> instances = reasoner.getInstances(owlClass, false);
            long endTime = System.nanoTime();
            times.add((endTime - startTime) / 1_000_000.0);

            if (i == 0) {
                for (Node<OWLNamedIndividual> node : instances) {
                    for (OWLNamedIndividual ind : node.getEntities()) {
                        instanceNames.add(ind.getIRI().getShortForm());
                    }
                }
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "instances");
        result.put("target_class", className);
        result.put("result_count", instanceNames.size());
        result.put("results", new ArrayList<>(instanceNames));
        result.put("timing_stats", calculateStats(times));
        queryTimings.put(queryId, times);
        queryResults.add(result);

        System.out.println("  " + queryId + ": p50=" + String.format("%.3f", calculatePercentile(times, 50)) + "ms");
    }

    private static void runUnsatisfiableQueryWithIterations(String queryId) {
        List<Double> times = new ArrayList<>();
        Set<String> unsatNames = new HashSet<>();

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            long startTime = System.nanoTime();
            Node<OWLClass> unsatisfiable = reasoner.getUnsatisfiableClasses();
            long endTime = System.nanoTime();
            times.add((endTime - startTime) / 1_000_000.0);

            if (i == 0) {
                for (OWLClass cls : unsatisfiable) {
                    if (!cls.isBottomEntity()) {
                        unsatNames.add(cls.getIRI().getShortForm());
                    }
                }
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "unsatisfiable");
        result.put("result_count", unsatNames.size());
        result.put("results", new ArrayList<>(unsatNames));
        result.put("timing_stats", calculateStats(times));
        queryTimings.put(queryId, times);
        queryResults.add(result);

        System.out.println("  " + queryId + ": p50=" + String.format("%.3f", calculatePercentile(times, 50)) + "ms");
    }

    private static void runTaxonomyDepthQueryWithIterations(String queryId, String className, String baseIRI) {
        List<Double> times = new ArrayList<>();
        int maxDepth = 0;

        OWLClass owlClass = dataFactory.getOWLClass(IRI.create(baseIRI + className));

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            long startTime = System.nanoTime();
            int depth = calculateMaxDepth(owlClass, 0, new HashSet<>());
            long endTime = System.nanoTime();
            times.add((endTime - startTime) / 1_000_000.0);

            if (i == 0) {
                maxDepth = depth;
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "taxonomy_depth");
        result.put("root_class", className);
        result.put("max_depth", maxDepth);
        result.put("timing_stats", calculateStats(times));
        queryTimings.put(queryId, times);
        queryResults.add(result);

        System.out.println("  " + queryId + ": p50=" + String.format("%.3f", calculatePercentile(times, 50)) + "ms");
    }

    private static Map<String, Object> calculateStats(List<Double> times) {
        Collections.sort(times);
        Map<String, Object> stats = new LinkedHashMap<>();

        double sum = 0;
        for (double t : times) sum += t;
        double mean = sum / times.size();

        double variance = 0;
        for (double t : times) variance += (t - mean) * (t - mean);
        double stddev = Math.sqrt(variance / times.size());

        stats.put("min_ms", times.get(0));
        stats.put("max_ms", times.get(times.size() - 1));
        stats.put("mean_ms", mean);
        stats.put("stddev_ms", stddev);
        stats.put("p50_ms", calculatePercentile(times, 50));
        stats.put("p90_ms", calculatePercentile(times, 90));
        stats.put("p95_ms", calculatePercentile(times, 95));
        stats.put("p99_ms", calculatePercentile(times, 99));
        stats.put("sample_count", times.size());

        return stats;
    }

    private static double calculatePercentile(List<Double> sortedTimes, int percentile) {
        if (sortedTimes.isEmpty()) return 0;
        List<Double> sorted = new ArrayList<>(sortedTimes);
        Collections.sort(sorted);
        int index = (int) Math.ceil(percentile / 100.0 * sorted.size()) - 1;
        index = Math.max(0, Math.min(index, sorted.size() - 1));
        return sorted.get(index);
    }

    // Keep original methods for backward compatibility (not used in multi-iteration mode)
    private static void runBenchmarkQueries() {
        String baseIRI = "http://flyby-robotics.com/ontology/uav#";

        // Query 1: Get all subclasses of UAV
        runSubclassQuery("Q1_UAV_Subclasses", "UAV", baseIRI);

        // Query 2: Get all subclasses of Multirotor
        runSubclassQuery("Q2_Multirotor_Subclasses", "Multirotor", baseIRI);

        // Query 3: Get all subclasses of UAVMission
        runSubclassQuery("Q3_Mission_Subclasses", "UAVMission", baseIRI);

        // Query 4: Get all subclasses of UAVSensor
        runSubclassQuery("Q4_Sensor_Subclasses", "UAVSensor", baseIRI);

        // Query 5: Get all subclasses of UAVAction
        runSubclassQuery("Q5_Action_Subclasses", "UAVAction", baseIRI);

        // Query 6: Get all subclasses of AirspaceRegion
        runSubclassQuery("Q6_Airspace_Subclasses", "AirspaceRegion", baseIRI);

        // Query 7: Check if FlybyF11 is subclass of Quadcopter
        runSubsumptionQuery("Q7_FlybyF11_Quadcopter", "FlybyF11", "Quadcopter", baseIRI);

        // Query 8: Check if Quadcopter is subclass of UAV
        runSubsumptionQuery("Q8_Quadcopter_UAV", "Quadcopter", "UAV", baseIRI);

        // Query 9: Get superclasses of FlybyF11
        runSuperclassQuery("Q9_FlybyF11_Superclasses", "FlybyF11", baseIRI);

        // Query 10: Get all instances of FlightPhase
        runInstanceQuery("Q10_FlightPhase_Instances", "FlightPhase", baseIRI);

        // Query 11: Get all instances of MissionStatus
        runInstanceQuery("Q11_MissionStatus_Instances", "MissionStatus", baseIRI);

        // Query 12: Get all instances of EmergencyCondition
        runInstanceQuery("Q12_Emergency_Instances", "EmergencyCondition", baseIRI);

        // Query 13: Get unsatisfiable classes (should be none)
        runUnsatisfiableQuery("Q13_Unsatisfiable_Classes");

        // Query 14: Full taxonomy depth for UAV hierarchy
        runTaxonomyDepthQuery("Q14_UAV_Taxonomy_Depth", "UAV", baseIRI);

        // Query 15: Check HoverCapable subclass hierarchy
        runSubclassQuery("Q15_HoverCapable_Subclasses", "HoverCapable", baseIRI);
    }

    private static void runSubclassQuery(String queryId, String className, String baseIRI) {
        long startTime = System.nanoTime();

        OWLClass owlClass = dataFactory.getOWLClass(IRI.create(baseIRI + className));
        NodeSet<OWLClass> subClasses = reasoner.getSubClasses(owlClass, false);

        long endTime = System.nanoTime();
        double timeMs = (endTime - startTime) / 1_000_000.0;

        Set<String> subClassNames = new HashSet<>();
        for (Node<OWLClass> node : subClasses) {
            for (OWLClass cls : node.getEntities()) {
                if (!cls.isBottomEntity()) {
                    String name = cls.getIRI().getShortForm();
                    subClassNames.add(name);
                }
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "subclass");
        result.put("target_class", className);
        result.put("result_count", subClassNames.size());
        result.put("results", new ArrayList<>(subClassNames));
        result.put("time_ms", timeMs);

        queryResults.add(result);
    }

    private static void runSuperclassQuery(String queryId, String className, String baseIRI) {
        long startTime = System.nanoTime();

        OWLClass owlClass = dataFactory.getOWLClass(IRI.create(baseIRI + className));
        NodeSet<OWLClass> superClasses = reasoner.getSuperClasses(owlClass, false);

        long endTime = System.nanoTime();
        double timeMs = (endTime - startTime) / 1_000_000.0;

        Set<String> superClassNames = new HashSet<>();
        for (Node<OWLClass> node : superClasses) {
            for (OWLClass cls : node.getEntities()) {
                if (!cls.isTopEntity()) {
                    String name = cls.getIRI().getShortForm();
                    superClassNames.add(name);
                }
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "superclass");
        result.put("target_class", className);
        result.put("result_count", superClassNames.size());
        result.put("results", new ArrayList<>(superClassNames));
        result.put("time_ms", timeMs);

        queryResults.add(result);
    }

    private static void runSubsumptionQuery(String queryId, String subClass, String superClass, String baseIRI) {
        long startTime = System.nanoTime();

        OWLClass sub = dataFactory.getOWLClass(IRI.create(baseIRI + subClass));
        OWLClass sup = dataFactory.getOWLClass(IRI.create(baseIRI + superClass));

        NodeSet<OWLClass> superClasses = reasoner.getSuperClasses(sub, false);
        boolean isSubsumed = false;
        for (Node<OWLClass> node : superClasses) {
            if (node.contains(sup)) {
                isSubsumed = true;
                break;
            }
        }

        long endTime = System.nanoTime();
        double timeMs = (endTime - startTime) / 1_000_000.0;

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "subsumption");
        result.put("subclass", subClass);
        result.put("superclass", superClass);
        result.put("is_subsumed", isSubsumed);
        result.put("time_ms", timeMs);

        queryResults.add(result);
    }

    private static void runInstanceQuery(String queryId, String className, String baseIRI) {
        long startTime = System.nanoTime();

        OWLClass owlClass = dataFactory.getOWLClass(IRI.create(baseIRI + className));
        NodeSet<OWLNamedIndividual> instances = reasoner.getInstances(owlClass, false);

        long endTime = System.nanoTime();
        double timeMs = (endTime - startTime) / 1_000_000.0;

        Set<String> instanceNames = new HashSet<>();
        for (Node<OWLNamedIndividual> node : instances) {
            for (OWLNamedIndividual ind : node.getEntities()) {
                instanceNames.add(ind.getIRI().getShortForm());
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "instances");
        result.put("target_class", className);
        result.put("result_count", instanceNames.size());
        result.put("results", new ArrayList<>(instanceNames));
        result.put("time_ms", timeMs);

        queryResults.add(result);
    }

    private static void runUnsatisfiableQuery(String queryId) {
        long startTime = System.nanoTime();

        Node<OWLClass> unsatisfiable = reasoner.getUnsatisfiableClasses();

        long endTime = System.nanoTime();
        double timeMs = (endTime - startTime) / 1_000_000.0;

        Set<String> unsatNames = new HashSet<>();
        for (OWLClass cls : unsatisfiable) {
            if (!cls.isBottomEntity()) {
                unsatNames.add(cls.getIRI().getShortForm());
            }
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "unsatisfiable");
        result.put("result_count", unsatNames.size());
        result.put("results", new ArrayList<>(unsatNames));
        result.put("time_ms", timeMs);

        queryResults.add(result);
    }

    private static void runTaxonomyDepthQuery(String queryId, String className, String baseIRI) {
        long startTime = System.nanoTime();

        OWLClass owlClass = dataFactory.getOWLClass(IRI.create(baseIRI + className));
        int maxDepth = calculateMaxDepth(owlClass, 0, new HashSet<>());

        long endTime = System.nanoTime();
        double timeMs = (endTime - startTime) / 1_000_000.0;

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("query_id", queryId);
        result.put("query_type", "taxonomy_depth");
        result.put("root_class", className);
        result.put("max_depth", maxDepth);
        result.put("time_ms", timeMs);

        queryResults.add(result);
    }

    private static int calculateMaxDepth(OWLClass cls, int currentDepth, Set<OWLClass> visited) {
        if (visited.contains(cls)) {
            return currentDepth;
        }
        visited.add(cls);

        NodeSet<OWLClass> subClasses = reasoner.getSubClasses(cls, true);
        int maxDepth = currentDepth;

        for (Node<OWLClass> node : subClasses) {
            for (OWLClass subClass : node.getEntities()) {
                if (!subClass.isBottomEntity()) {
                    int depth = calculateMaxDepth(subClass, currentDepth + 1, visited);
                    maxDepth = Math.max(maxDepth, depth);
                }
            }
        }

        return maxDepth;
    }

    private static void writeResults(String path) throws IOException {
        try (PrintWriter writer = new PrintWriter(new FileWriter(path))) {
            writeJsonObject(writer, results, 0);
        }
    }

    @SuppressWarnings("unchecked")
    private static void writeJsonObject(PrintWriter writer, Object obj, int indent) {
        String indentStr = "  ".repeat(indent);
        String nextIndent = "  ".repeat(indent + 1);

        if (obj instanceof Map) {
            Map<String, Object> map = (Map<String, Object>) obj;
            writer.println("{");
            int i = 0;
            for (Map.Entry<String, Object> entry : map.entrySet()) {
                writer.print(nextIndent + "\"" + entry.getKey() + "\": ");
                writeJsonValue(writer, entry.getValue(), indent + 1);
                if (i < map.size() - 1) {
                    writer.println(",");
                } else {
                    writer.println();
                }
                i++;
            }
            writer.print(indentStr + "}");
        } else if (obj instanceof List) {
            List<?> list = (List<?>) obj;
            if (list.isEmpty()) {
                writer.print("[]");
            } else {
                writer.println("[");
                for (int i = 0; i < list.size(); i++) {
                    writer.print(nextIndent);
                    writeJsonValue(writer, list.get(i), indent + 1);
                    if (i < list.size() - 1) {
                        writer.println(",");
                    } else {
                        writer.println();
                    }
                }
                writer.print(indentStr + "]");
            }
        }
    }

    @SuppressWarnings("unchecked")
    private static void writeJsonValue(PrintWriter writer, Object value, int indent) {
        if (value == null) {
            writer.print("null");
        } else if (value instanceof String) {
            writer.print("\"" + escapeJson((String) value) + "\"");
        } else if (value instanceof Number) {
            writer.print(value);
        } else if (value instanceof Boolean) {
            writer.print(value);
        } else if (value instanceof Map || value instanceof List) {
            writeJsonObject(writer, value, indent);
        } else {
            writer.print("\"" + escapeJson(value.toString()) + "\"");
        }
    }

    private static String escapeJson(String s) {
        return s.replace("\\", "\\\\")
                .replace("\"", "\\\"")
                .replace("\n", "\\n")
                .replace("\r", "\\r")
                .replace("\t", "\\t");
    }
}
